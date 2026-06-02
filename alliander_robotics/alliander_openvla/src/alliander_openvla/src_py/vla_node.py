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
from alliander_openvla.model_manager import ModelManager
from alliander_openvla.utils import image_to_pil, cv_to_ros_image, convert_posedelta_to_velo
# from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TwistStamped, PoseStamped
from moveit_msgs.srv import ServoCommandType
from PIL import Image as PILImage
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
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
        self.current_task = tasks_config.get("reach_to_object", list(self.tasks.keys())[0])

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
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/vla/waypoint_markers",
            10
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
        self.traj_points = []
        self.max_traj_length = 50
        self.current_gripper_state = None
        self.gripper_busy = False
        self.current_target_pose = None
        # Short debug timer
        # self.debug_timer = self.create_timer(30.0, self.send_debug_pose)


        # Performance tracking timer
        self.create_timer(5.0, self.publish_metrics)
        self.get_logger().info("VLA Node ready!")
        self.get_logger().info(f"Subscribed to: {camera_topic}")
        # self.get_logger().info("Publishing to: /vla/waypoint_markers, /vla/annotated_image")
        # self.set_command_via_params(command_mode)
        self.set_command_mode(command_mode)

    
    
    def send_debug_pose(self):
        self.get_logger().info("Sending DEBUG pose command")

        # Big, visible motion (10 cm forward in EE frame)
        action = np.array([0.10, 0.0, 0.0, 0.0, 0.0, 0.0])

        try:
            pose_msg = self.create_pose_msg(action)
            self.current_target_pose = pose_msg

            self.publish_pose_action(pose_msg)

            # Also show marker
            if self.get_parameter("enable_visualization").value:
                self.publish_markers(action)

        except Exception as e:
            self.get_logger().error(f"Debug pose failed: {e}")

    
    def set_command_via_params(self, command_mode) -> None:
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt Servo parameter service to start...')

        # 3. Construct the parameter request payload
        req = SetParameters.Request()
        param_value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value="speed_units")
        req.parameters = [Parameter(name="command_in_type", value=param_value)]

        # 4. Fire the service call asynchronously
        self.get_logger().info('Changing MoveIt Servo command type via parameter service...')
        future = self.param_client.call_async(req)
        # future.add_done_callback(self._handle_switch_response)
    
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

    def publish_raw_action(self, action: np.ndarray) -> None:
        """Publish raw predicted action."""
        msg = Float32MultiArray()
        msg.data = action.tolist()
        #self.get_logger().info("Publishing action...")
        # Debug line
        self.get_logger().info(f"action: {action}")
        self.raw_action_pub.publish(msg)

    def publish_twist_action(self, twist_msg: TwistStamped) -> None:
        """Publish predicted action to servo_node as TwistStamped."""
        self.twist_action_pub.publish(twist_msg)

    def publish_pose_action(self, pose_msg: PoseStamped):
        """Publish predicted action to servo_node as PoseStamped."""
        self.pose_action_pub.publish(pose_msg)

    def publish_markers(self, action: np.ndarray) -> None:
        """Publish 3D markers for RViz based on EE delta actions."""

        marker_array = MarkerArray()

        # Lookup current EE pose
        try:
            transform = self.tf_buffer.lookup_transform(
                "franka/world",                  # target frame
                "franka/fr3_hand_tcp",           # EE frame
                rclpy.time.Time()
            )

            ee_x = transform.transform.translation.x
            ee_y = transform.transform.translation.y
            ee_z = transform.transform.translation.z

            rot = transform.transform.rotation
            current_rot = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w])


        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return  # Don't publish broken markers

        # Compute target from deltas        
        dx, dy, dz = action[:3]
        delta_world = current_rot.apply([dx, dy, dz])

        target_x = ee_x + delta_world[0]
        target_y = ee_y + delta_world[1]
        target_z = ee_z + delta_world[2]


        # Arrow: EE --> Target
        arrow = Marker()
        arrow.header.frame_id = "franka/world"
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = "vla_waypoints"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        # REQUIRED when using points
        arrow.pose.orientation.w = 1.0

        # Define arrow start and end
        arrow.points = [
            Point(x=ee_x, y=ee_y, z=ee_z),
            Point(x=target_x, y=target_y, z=target_z)
        ]

        # Arrow thickness
        arrow.scale.x = 0.02  # shaft diameter
        arrow.scale.y = 0.04  # head diameter
        arrow.scale.z = 0.08  # head length

        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0

        arrow.lifetime.sec = 0

        marker_array.markers.append(arrow)

        # Optional: EE sphere (nice debug)
        ee_marker = Marker()
        ee_marker.header = arrow.header
        ee_marker.ns = "ee_position"
        ee_marker.id = 2
        ee_marker.type = Marker.SPHERE
        ee_marker.action = Marker.ADD

        ee_marker.pose.position.x = ee_x
        ee_marker.pose.position.y = ee_y
        ee_marker.pose.position.z = ee_z
        ee_marker.pose.orientation.w = 1.0

        ee_marker.scale.x = 0.04
        ee_marker.scale.y = 0.04
        ee_marker.scale.z = 0.04

        ee_marker.color.r = 0.0
        ee_marker.color.g = 0.0
        ee_marker.color.b = 1.0
        ee_marker.color.a = 1.0

        marker_array.markers.append(ee_marker)

        # Optional: Target sphere (nice debug)
        target_marker = Marker()
        target_marker.header = arrow.header
        target_marker.ns = "target_position"
        target_marker.id = 3
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD

        target_marker.pose.position.x = target_x
        target_marker.pose.position.y = target_y
        target_marker.pose.position.z = target_z
        target_marker.pose.orientation.w = 1.0

        target_marker.scale.x = 0.04
        target_marker.scale.y = 0.04
        target_marker.scale.z = 0.04

        target_marker.color.r = 1.0
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        target_marker.color.a = 1.0

        marker_array.markers.append(target_marker)

        # Text label at target
        text_marker = Marker()
        text_marker.header = arrow.header
        text_marker.ns = "vla_labels"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = target_x
        text_marker.pose.position.y = target_y
        text_marker.pose.position.z = target_z + 0.1
        text_marker.pose.orientation.w = 1.0

        text_marker.scale.z = 0.05
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = self.current_task

        marker_array.markers.append(text_marker)

        # Add trajectory plot
        self.traj_points.append(Point(x=target_x, y=target_y, z=target_z))

        if len(self.traj_points) > self.max_traj_length:
            self.traj_points.pop(0) # Keep buffer limited

        traj_marker = Marker()
        traj_marker.header = arrow.header
        traj_marker.ns = "trajectory"
        traj_marker.id = 4
        traj_marker.type = Marker.LINE_STRIP
        traj_marker.action = Marker.ADD

        traj_marker.points = self.traj_points

        traj_marker.scale.x = 0.01  # line width

        traj_marker.color.r = 1.0
        traj_marker.color.g = 1.0
        traj_marker.color.b = 0.0
        traj_marker.color.a = 1.0

        marker_array.markers.append(traj_marker)

        # Debug logging
        self.get_logger().info(
            f"EE: ({ee_x:.2f}, {ee_y:.2f}, {ee_z:.2f}) | "
            f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
        )

        # Publish
        self.marker_pub.publish(marker_array)

    def publish_annotated_image(self, cv_image: np.ndarray, action: np.ndarray, result: dict) -> None:
        """Publish annotated 2D image."""
        annotated = cv_image.copy()
        h, w = annotated.shape[:2]

        # Draw target point (project 3D to 2D - simplified)
        target_x = int((action[0] + 1) * w / 2)
        target_y = int((action[1] + 1) * h / 2)

        # Draw crosshair
        cv2.circle(annotated, (target_x, target_y), 10, (0, 255, 0), 2)
        cv2.line(annotated, (target_x - 15, target_y), (target_x + 15, target_y), (0, 255, 0), 2)
        cv2.line(annotated, (target_x, target_y - 15), (target_x, target_y + 15), (0, 255, 0), 2)

        # Add text overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(annotated, f"Task: {self.current_task}", (10, 30),
                    font, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated, f'Inference: {result["inference_time"] * 1000:.1f}ms',
                    (10, 60), font, 0.6, (255, 255, 255), 1)
        cv2.putText(annotated, f"Target: ({action[0]:.2f}, {action[1]:.2f}, {action[2]:.2f})",
                    (10, 90), font, 0.6, (255, 255, 255), 1)

        # Convert back to ROS message
        annotated_msg = cv_to_ros_image(annotated)
        annotated_msg.header.stamp = self.get_clock().now().to_msg()
        annotated_msg.header.frame_id = "camera"

        self.get_logger().info("Publishing annotated image...")
        self.annotated_image_pub.publish(annotated_msg)

    def publish_status(self, status: str) -> None:
        """Publish status message."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def publish_metrics(self) -> None:
        """Publish performance metrics."""
        metrics = self.model_manager.get_metrics()
        metrics["dropped_frames"] = self.dropped_frames
        metrics["total_frames"] = self.frame_count
        metrics["drop_rate"] = self.dropped_frames / max(self.frame_count, 1)

        msg = String()
        msg.data = yaml.dump(metrics)
        self.metrics_pub.publish(msg)

        # Log summary
        self.get_logger().info(
            f'Metrics: {metrics["fps"]:.2f} FPS, '
            f'{metrics["drop_rate"] * 100:.1f}% dropped, '
            f'{metrics["avg_inference_time"] * 1000:.1f}ms avg'
        )

    def process_frame(self, msg, framecount) -> None:
        # if hasattr(self, "debug_timer"):
        #         return

        try:
            # Convert ROS image to OpenCV
            # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

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

            # Convert to PIL for model without using cv_bridge
            # try:
            #     pil_image = image_to_pil(msg)

            # except Exception as e:
            #     self.get_logger().error(f"Image processing failed: {e}")

            # Get current task
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
            # action = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            self.last_action = action
            self.publish_raw_action(action)

            # Set gripper according to action array length
            if len(action) > 6:
                gripper = action[6]
            else:
                gripper = None

            # Check command mode and perform corresponding actions
            command_mode = self.get_parameter("command_mode").value
            if command_mode == "twist":
                linear, angular = convert_posedelta_to_velo(action)
                twist_msg = self.create_twist_msg(linear, angular)
                self.publish_twist_action(twist_msg)
                # + Publish gripper action?
                self.handle_gripper(gripper)
                # + Set twist to 0 again
                # twist_stop_msg = self.create_twist_msg((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
                # self.publish_twist_action(twist_stop_msg)
            elif command_mode == "pose":
                pose_msg = self.create_pose_msg(action)
                self.current_target_pose = pose_msg
                self.publish_pose_action(pose_msg)
                # + Publish gripper action?
                self.handle_gripper(gripper)

            
            # Publish visualization
            if self.get_parameter("enable_visualization").value:
                self.publish_markers(action)
                self.publish_annotated_image(cv_image, action, result)

            # Log performance
            self.get_logger().info(
                f'Processed frame {framecount} in {result["inference_time"] * 1000:.1f}ms'
            )

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

        finally:
            self.processing = False

    
    def handle_gripper(self, gripper_value):
        if gripper_value is None:
            return

        # Prevent sending new commands while busy
        if self.gripper_busy:
            return

        # Hysteresis (VERY important for stability)
        open_thresh = 0.2
        close_thresh = -0.2

        if gripper_value > open_thresh:
            desired = "open"
        elif gripper_value < close_thresh:
            desired = "close"
        else:
            return

        # Avoid repeated triggering
        if desired == self.current_gripper_state:
            return

        self.current_gripper_state = desired

        if desired == "open":
            client = self.gripper_open_client
        else:
            client = self.gripper_close_client

        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(f"{desired} action server not available")
            return

        goal = TriggerAction.Goal()  # usually empty

        self.get_logger().info(f"Gripper: {desired}")

        self.gripper_busy = True # Set busy flag
        future = client.send_goal_async(goal)
        future.add_done_callback(self.gripper_goal_response_callback)

        # Reset busy flag after a certain time; we assume the gripper action is correctly executed.
        # Setting a callback on the action's result caused timeouts.
        reset_time = 0.5 if desired =="open" else 1.0

        self.gripper_timer = self.create_timer(
            reset_time,
            self.reset_gripper_busy_once
        )

    
    def reset_gripper_busy_once(self):
        self.gripper_busy = False
        self.get_logger().info("Gripper ready again")

        # Cancel timer
        if self.gripper_timer is not None:
            self.gripper_timer.cancel()
            self.gripper_timer = None


    
    def gripper_goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Gripper goal rejected")
            self.gripper_busy = False
            return

        self.get_logger().info("Gripper goal accepted")

        # result_future = goal_handle.get_result_async()
        # result_future.add_done_callback(self.gripper_result_callback)


    # def gripper_result_callback(self, future) -> None:
    #     try:
    #         result = future.result().result
    #         self.get_logger().info("Gripper action completed")
    #     except Exception as e:
    #         self.get_logger().error(f"Gripper result error: {e}")


    
    def check_pose_goal_reached(self):
        if self.current_target_pose is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "franka/world",
                "franka/fr3_hand_tcp",
                rclpy.time.Time()
            )

            ee = transform.transform.translation
            target = self.current_target_pose.pose.position

            dist = np.sqrt(
                (ee.x - target.x)**2 +
                (ee.y - target.y)**2 +
                (ee.z - target.z)**2
            )

            if dist < 0.02:  # 2 cm tolerance
                self.get_logger().info("Target reached.")
                self.stop_robot()
                self.current_target_pose = None

        except Exception as e:
            self.get_logger().warn(f"Goal check failed: {e}")



    def create_twist_msg(self, linear, angular):
        twist = TwistStamped()

        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "franka/fr3_hand_tcp"   # important! ToDo: Check whether this is right. "franka/world" is base frame

        twist.twist.linear.x = linear[0]
        twist.twist.linear.y = linear[1]
        twist.twist.linear.z = linear[2]

        twist.twist.angular.x = angular[0]
        twist.twist.angular.y = angular[1]
        twist.twist.angular.z = angular[2]

        return twist
    
    def create_pose_msg(self, action: np.ndarray):
        pose_msg = PoseStamped()

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "franka/world"  # same as twist case TODO: Should this be world or EE frame?

        dx, dy, dz = action[:3]
        droll, dpitch, dyaw = action[3:6]
        
        # Get current EE pose
        try:
            transform = self.tf_buffer.lookup_transform(
                "franka/world",
                "franka/fr3_hand_tcp",
                rclpy.time.Time()
            )

            pos = transform.transform.translation
            rot = transform.transform.rotation

            current_rot = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w])
            delta_rot = Rotation.from_euler("xyz", [droll, dpitch, dyaw])

            target_rot = current_rot * delta_rot

            quat = target_rot.as_quat()

            # Apply position delta (EE frame → rotate into world!)
            delta_world = current_rot.apply([dx, dy, dz])

            pose_msg.pose.position.x = pos.x + delta_world[0]
            pose_msg.pose.position.y = pos.y + delta_world[1]
            pose_msg.pose.position.z = pos.z + delta_world[2]

            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

        except Exception as e:
            self.get_logger().warn(f"Pose transform failed: {e}")

        return pose_msg


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
