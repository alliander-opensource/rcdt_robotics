#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import cv2
import threading
import numpy as np
import rclpy
import torch
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from src_py.motion_manager import MotionManager
from src_py.gripper_manager import GripperManager
from src_py.visualization_manager import VisualizationManager
from alliander_openvla.model_manager import ModelManager
from alliander_openvla.utils import image_to_pil, cv_to_ros_image, convert_posedelta_to_velo
# from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TwistStamped, PoseStamped
from moveit_msgs.srv import ServoCommandType
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from alliander_utilities.ros_utils import get_file_path, get_yaml
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rclpy.action import ActionClient
from alliander_interfaces.action import TriggerAction



# For this code, heavy inspiration was taken from https://github.com/DeeptamBhar/OpenVLA_ROS2/blob/main/demos/ros2_integration/ros2_ws/src/vla_control/vla_controller_node.py
class VLANode(Node):
    """Node containing the logic for the Vision Language Action (VLA) model."""

    def __init__(self):
        """"Initialize the VLA Node."""
        super().__init__("vla_node")

        # Params
        # ToDo: Look into and add model config file
        self.declare_parameter("model_config", get_file_path("alliander_openvla", ["config"], "model_config.yaml"))
        self.declare_parameter("tasks_config", get_file_path("alliander_openvla", ["config"], "tasks.yaml"))
        self.declare_parameter("camera_topic", "/realsense/color/image_raw")
        self.declare_parameter("max_queue_size", 2)
        self.declare_parameter("enable_visualization", True)
        self.declare_parameter("command_mode", "pose") # Parameter to control which command type the model outputs: set to ...

        # Get parameters
        model_config_path = self.get_parameter("model_config").value
        tasks_config_path = self.get_parameter("tasks_config").value
        camera_topic = self.get_parameter("camera_topic").value
        max_queue_size = self.get_parameter("max_queue_size").value
        command_mode = self.get_parameter("command_mode").value
        
        valid_modes = ["twist", "pose"]

        if command_mode not in valid_modes:
            self.get_logger().error(
                f"Invalid command_mode '{command_mode}'. Must be one of {valid_modes}."
            )
            raise ValueError(f"Unsupported command_mode: {command_mode}")


        self.get_logger().info("Initializing VLA Node...")
        # ToDo: Research how exactly I will define tasks
        # Load tasks configuration
        tasks_config = get_yaml(tasks_config_path)
        self.tasks = tasks_config["tasks"]
        self.current_task = tasks_config.get("default_task", list(self.tasks.keys())[0])

        self.get_logger().info(f"Loaded {len(self.tasks)} tasks")
        self.get_logger().info(f"Current task: {self.current_task}")

        self.get_logger().info(f"Torch version:{torch.version}")
        self.get_logger().info(f"Torch version:{torch.__version__}")

        # Load OpenVLA model
        # Initialize model manager
        self.get_logger().info("Loading VLA model...")
        self.model_manager = ModelManager(model_config_path)
        if not self.model_manager.load_model():
            self.get_logger().error("Failed to load model!")
            raise RuntimeError("Model loading failed")
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # # CV Bridge
        # self.bridge = CvBridge()

        # Subscribe to image from realsense camera
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            max_queue_size
        )
        self.task_sub = self.create_subscription(
            String,
            "/vla/task_command",
            self.task_callback,
            10
        )
        # Publishers
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/vla/waypoint_markers",
            qos_profile
        )

        self.annotated_image_pub = self.create_publisher(
            Image,
            "/vla/annotated_image",
            10
        )
        self.raw_action_pub = self.create_publisher(
            Float32MultiArray,
            "/vla/raw_predicted_action",
            10
        )
        # Added two publishers to servo node that publish TwistStamped or PoseStamped messages. ToDo: check values etc.
        self.twist_action_pub = self.create_publisher(
            TwistStamped,
            "/franka/servo_node/delta_twist_cmds",
            10
        )
        self.pose_action_pub = self.create_publisher(
            PoseStamped,
            "/franka/servo_node/pose_target_cmds",
            10
        )
        self.status_pub = self.create_publisher(
            String,
            "/vla/status",
            10
        )
        self.metrics_pub = self.create_publisher(
            String,
            "/vla/metrics",
            10
        )
        # Client for switching moveit command type
        self.switch_cmd_type = self.create_client(
            ServoCommandType,
            "/franka/servo_node/switch_command_type"
        )
        # # Client for switching moveit command type using the servo node's parameter
        # self.param_client - self.create_client(
        #     SetParameters,
        #     '/franka/servo_node/set_parameters'
        # )
        self.gripper_open_client = ActionClient(
            self,
            TriggerAction,
            "/franka/gripper/open"
        )

        self.gripper_close_client = ActionClient(
            self,
            TriggerAction,
            "/franka/gripper/close"
        )

        # State
        self.processing = False
        self.last_action = None
        self.frame_count = 0
        self.dropped_frames = 0
        # Short debug timer
        self.debug_timer = self.create_timer(30.0, self.send_debug_pose)

        self.motion_manager = MotionManager(
            node=self,
            pose_pub=self.pose_action_pub,
            twist_pub=self.twist_action_pub,
            tf_buffer=self.tf_buffer
        )
        self.gripper_manager = GripperManager(
            node=self,
            open_client=self.gripper_open_client,
            close_client=self.gripper_close_client
        )
        self.vis_manager = VisualizationManager(
            node=self,
            marker_pub=self.marker_pub,
            image_pub=self.annotated_image_pub
        )


        # Performance tracking timer
        self.create_timer(5.0, self.publish_metrics)
        self.get_logger().info("VLA Node ready!")
        self.get_logger().info(f"Subscribed to: {camera_topic}")
        # self.get_logger().info("Publishing to: /vla/waypoint_markers, /vla/annotated_image")
        self.set_command_mode(command_mode)

    
    def send_debug_pose(self):
        self.get_logger().info("Sending DEBUG pose command")

        # Big, visible motion (10 cm forward in EE frame)
        action = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        command_mode = self.get_parameter("command_mode").value

        ee_pose, current_rot = self.get_ee_pose()
        if ee_pose is None:
            return
        
        target_pose = self.motion_manager.execute_action(action, command_mode)
        vis_target = self.compute_visual_target(
            action,
            ee_pose,
            current_rot,
            target_pose
        )

        # Publish visualization is enabled
        if self.get_parameter("enable_visualization").value:
            self.vis_manager.publish_markers(
                ee_pose,
                vis_target,
                action,
                self.current_task
            )

            # self.vis_manager.publish_annotated_image(
            #     cv_image,
            #     action,
            #     result,
            # )


    
    def _handle_switch_response(self, future):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info("Servo command type swtiched.")
            else:
                self.get_logger().warn("Failed to set servo command type.")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def set_command_mode(self, mode: str) -> None:
        if mode == "twist":
            self.get_logger().info(f"Switching Servo command type to {mode}...")
            self.switch_servo(1)
        elif mode == "pose":
            self.get_logger().info(f"Switching Servo command type to {mode}...")
            self.switch_servo(2)
    
    
    def switch_servo(self, command_type: int) -> None:
        """"Function to switch the servo_node's command_in_type
        
        Args:
            command_type (int): The command_in_type to switch to. 0=JOINT, 1=TWIST, 2=POSE.
        """
        if not self.switch_cmd_type.service_is_ready():
            self.get_logger().warn("Switch command type service not available.")
            return
        else:
            self.get_logger().info("Switch command type service is available")

        self.get_logger().info("Initializing request...")
        request = ServoCommandType.Request()

        self.get_logger().info(f"Setting request's command type to {command_type}")
        request.command_type = command_type

        self.get_logger().info("Calling request to the service...")
        future = self.switch_cmd_type.call_async(request) # I get bytes error from this line...
        self.get_logger().info("Request to service made!")

        future.add_done_callback(self._handle_switch_response)


    # Function to handle task switching, assumes a task.yaml file. Will have to see how I represent the tasks
    def task_callback(self, msg: String) -> None:
        """Handle task switching commands."""
        task_cmd = msg.data.strip()

        # Parse command (format: "switch:<task_name>" or "set_params:<key>=<value>")
        if task_cmd.startswith("switch:"):
            new_task = task_cmd.split(":", 1)[1]
            if new_task in self.tasks:
                self.current_task = new_task
                self.get_logger().info(f"Switched to task: {new_task}")
                self.publish_status(f"Task switched to: {new_task}")
            else:
                self.get_logger().warn(f"Unknown task: {new_task}")
                self.publish_status(f"Error: Unknown task {new_task}")

        elif task_cmd.startswith("list_tasks"):
            task_list = ", ".join(self.tasks.keys())
            self.get_logger().info(f"Available tasks: {task_list}")
            self.publish_status(f"Available tasks: {task_list}")

        elif task_cmd.startswith("current_task"):
            self.publish_status(f"Current task: {self.current_task}")

        else:
            self.get_logger().warn(f"Unknown command: {task_cmd}")


    def image_callback(self, msg: Image) -> None:
        """Process incoming images."""
        self.frame_count += 1

        # Drop frames if still processing
        if self.processing:
            self.dropped_frames += 1
            if self.dropped_frames % 10 == 0:
                self.get_logger().warn(
                    f"Dropped {self.dropped_frames} frames due to slow inference"
                )
            return

        self.processing = True

        # Added threading
        framecount = self.frame_count
        threading.Thread(target=self.process_frame, args=(msg, framecount)).start()


    def parse_action(self, action: np.ndarray) -> np.ndarray:
        """Parse action string to numpy array. This is a simplified version - adjust based on actual model output.

        Returns:
            array of actions
        """
        # For demo purposes, generate random actions
        # In production, parse the actual model output
        task = self.tasks[self.current_task]
        action_dims = task["action_dims"]
        
        # Example: Random action in normalized space [-1, 1]
        # action = np.random.uniform(-1, 1, action_dims)

        return action
    #TODO: this function does nothing currently... remove altogether?


    def publish_raw_action(self, action: np.ndarray) -> None:
        """Publish raw predicted action."""
        msg = Float32MultiArray()
        msg.data = action.tolist()
        #self.get_logger().info("Publishing action...")
        # Debug line
        self.get_logger().info(f"action: {action}")
        self.raw_action_pub.publish(msg)


    def publish_status(self, status: str) -> None:
        """Publish status message."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


    def publish_metrics(self) -> None:
        """Publish performance metrics."""

        # Get model metrics
        metrics = self.model_manager.get_metrics()

        # Add node-level metrics
        metrics["dropped_frames"] = self.dropped_frames
        metrics["total_frames"] = self.frame_count
        metrics["drop_rate"] = self.dropped_frames / max(self.frame_count, 1)

        # Convert to string (YAML format for readability)
        msg = String()
        msg.data = yaml.dump(metrics)

        # Publish
        self.metrics_pub.publish(msg)

        # Log summary (safe access with defaults)
        fps = metrics.get("fps", 0.0)
        drop_rate = metrics.get("drop_rate", 0.0)
        avg_inf = metrics.get("avg_inference_time", 0.0)

        self.get_logger().info(
            f"Metrics: {fps:.2f} FPS, "
            f"{drop_rate * 100:.1f}% dropped, "
            f"{avg_inf * 1000:.1f} ms avg"
        )


    def process_frame(self, msg, framecount) -> None:
        if hasattr(self, "debug_timer"):
                return

        try:
            # Convert ROS --> numpy --> PIL 
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)

            if msg.encoding == "rgb8":
                cv_image = np_arr.reshape((msg.height, msg.width, 3))
                pil_image = PILImage.fromarray(cv_image, "RGB")

            elif msg.encoding == "bgr8":
                cv_image = np_arr.reshape((msg.height, msg.width, 3))
                pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

            else:
                raise ValueError(f"Unsupported encoding: {msg.encoding}")

            task = self.tasks[self.current_task]

            # Create prompt (with example substitutions)
            instruction = task["prompt"].format(
                object="red block",
                container="blue bin",
                target="left",
                drawer="top"
            )
            prompt = f"In: What action should the robot take to {instruction}?\nOut:"
            self.get_logger().info(f'Input prompt: {prompt}')

            # Run inference
            result = self.model_manager.infer(pil_image, prompt, task=self.current_task)

            if result is None:
                self.get_logger().error("Inference failed")
                self.processing = False
                return
            else:
                self.get_logger().info("Inference finished, processing result...")

            # Parse action (this is simplified - real parsing depends on model output format)
            action = self.parse_action(result["action"])
            # action = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
            
            # Scale action for testing; robot moves further...
            action[:6] = action[:6] * 10
            #TODO: Add safety action check that caps them at a certain value such that it cannot break
            self.last_action = action
            self.publish_raw_action(action)

            # Extract End-effector position
            ee_pose, current_rot = self.get_ee_pose()

            if ee_pose is None:
                return
            
            # Check command mode and perform corresponding actions
            command_mode = self.get_parameter("command_mode").value
            # Use ModelManager class instance to execute the action and publish the message to servo_node.
            # If model is set to twist, the target_pose returned is `None`
            target_pose = self.motion_manager.execute_action(action, command_mode)

            # Set gripper according to task definition and execute gripper movement
            action_dims = task["action_dims"]

            if action_dims > 6:
                gripper_value = action[6]
                self.gripper_manager.handle(gripper_value)

            # Check if ee_pose and current_rot exist
            if ee_pose is None or current_rot is None:
                return
            # Compute target pose for Rviz visualization
            vis_target = self.compute_visual_target(
                action,
                ee_pose,
                current_rot,
                target_pose
            )
            
            # Publish visualization is enabled
            if self.get_parameter("enable_visualization").value:
                self.vis_manager.publish_markers(
                    ee_pose,
                    vis_target,
                    action,
                    self.current_task
                )

                self.vis_manager.publish_annotated_image(
                    cv_image,
                    action,
                    result,
                )

            # Log performance
            self.get_logger().info(
                f'Processed frame {framecount} in {result["inference_time"] * 1000:.1f}ms'
            )

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

        finally:
            self.processing = False

    
    def get_ee_pose(self):
        """Get end-effector pose and rotation."""
        try:
            transform = self.tf_buffer.lookup_transform(
                "franka/world",
                "franka/fr3_hand_tcp",
                rclpy.time.Time()
            )

            # Position
            t = transform.transform.translation
            ee_pose = (t.x, t.y, t.z)

            # Rotation
            r = transform.transform.rotation
            current_rot = Rotation.from_quat([r.x, r.y, r.z, r.w])

            return ee_pose, current_rot

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None, None
        
    
    def compute_visual_target(self, action, ee_pose, current_rot, target_pose):
        """Compute target position for visualization (pose or twist mode)."""

        if target_pose is not None: # Case: pose (only pose returns the target_pose)
            return (
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z
            )

        # Fallback: compute from delta action (twist mode)
        dx, dy, dz = action[:3]
        delta_world = current_rot.apply([dx, dy, dz])

        return (
            ee_pose[0] + delta_world[0],
            ee_pose[1] + delta_world[1],
            ee_pose[2] + delta_world[2]
        )



def main(args: list | None = None) -> None:
    """Main function to initialize the VLA node and control the robot accordingly."""
    rclpy.init(args=args)
    node = VLANode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.model_manager.print_metrics()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
