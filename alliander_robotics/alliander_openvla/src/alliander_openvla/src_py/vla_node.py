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
from geometry_msgs.msg import Point, TwistStamped
# from moveit_msgs.srv import ServoCommandType
from PIL import Image as PILImage
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from alliander_utilities.ros_utils import get_file_path, get_yaml


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

        # Get parameters
        model_config_path = self.get_parameter("model_config").value
        tasks_config_path = self.get_parameter("tasks_config").value
        camera_topic = self.get_parameter("camera_topic").value
        max_queue_size = self.get_parameter("max_queue_size").value

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
        # Added publisher to servo node that publishes twiststamped messages. ToDo: check values etc.
        self.twist_action_pub = self.create_publisher(
            TwistStamped,
            "/franka/servo_node/delta_twist_cmds",
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
        # # Client for switching moveit command type
        # self.switch_cmd_type = self.create_client(
        #     ServoCommandType,
        #     "/franka/servo_node/switch_command_type"
        # )

        # State
        self.processing = False
        self.last_action = None
        self.frame_count = 0
        self.dropped_frames = 0

        # Performance tracking timer
        self.create_timer(5.0, self.publish_metrics)
        self.get_logger().info("VLA Node ready!")
        self.get_logger().info(f"Subscribed to: {camera_topic}")
        # self.get_logger().info("Publishing to: /vla/waypoint_markers, /vla/annotated_image")
        # self.switch_servo()

    
    # def _handle_switch_response(self, future):
    #     try:
    #         response = future.result()

    #         if response.success:
    #             self.get_logger().info("Servo command type swtiched.")
    #         else:
    #             self.get_logger().warn("Failed to set servo command type.")

    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed: {e}")


    # def switch_servo(self, command_type: int=1) -> None:
    #     """"Function to switch the servo_node's command_in_type
        
    #     Args:
    #         command_type (int): The command_in_type to switch to. 0=JOINT, 1=TWIST, 2=POSE. Defaults to 1 (Twist).
    #     """
    #     if not self.switch_cmd_type.service_is_ready():
    #         self.get_logger().warn("Switch command type service not available.")
    #         return
        
    #     request = ServoCommandType.Request()

    #     request.command_type = command_type

    #     future = self.switch_cmd_type.call_async(request)

    #     future.add_done_callback(self._handle_switch_response)


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

    def publish_markers(self, action: np.ndarray) -> None:
        """Publish 3D markers for RViz."""
        marker_array = MarkerArray()

        # Create arrow marker for end-effector target
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vla_waypoints"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Position (scale from normalized [-1,1] to world coordinates)
        marker.pose.position.x = float(action[0] * 0.5)  # Scale to 0.5m range
        marker.pose.position.y = float(action[1] * 0.5)
        marker.pose.position.z = float(action[2] * 0.5 + 0.5)  # Offset z

        # Orientation (if available)
        if len(action) >= 6:
            r = Rotation.from_euler("xyz", action[3:6])
            quat = r.as_quat()
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
        else:
            marker.pose.orientation.w = 1.0

        # Scale and color
        marker.scale.x = 0.2  # Arrow length
        marker.scale.y = 0.02  # Arrow width
        marker.scale.z = 0.02  # Arrow height

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0  # Persist until replaced

        marker_array.markers.append(marker)

        # Add text label
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = "vla_labels"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = marker.pose.position.x
        text_marker.pose.position.y = marker.pose.position.y
        text_marker.pose.position.z = marker.pose.position.z + 0.1
        text_marker.scale.z = 0.05
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = self.current_task

        marker_array.markers.append(text_marker)

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
            prompt = task["prompt"].format(
                object="red cup",
                container="blue bin",
                target="left",
                drawer="top"
            )

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
            self.last_action = action

            # Convert model output to usable velocities for twist commands
            linear, angular, gripper = convert_posedelta_to_velo(action)

            # Create twist command from velocities
            twist_msg = self.create_twist_msg(linear, angular)

            # Publish action (raw + twist)
            self.publish_raw_action(action)
            self.publish_twist_action(twist_msg)

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
