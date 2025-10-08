#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
import rclpy
from graspnetpy_utils import data_utils
from rcdt_messages.msg import Grasp
from rcdt_messages.srv import (
    AddMarker,
    DefineGoalPose,
    GenerateGraspnetGrasp,
    MoveHandToPose,
    MoveToConfiguration,
)
from rcdt_utilities.launch_utils import LaunchArgument, spin_node
from rclpy import logging
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

ros_logger = logging.get_logger(__name__)
namespace_arm = LaunchArgument("namespace_arm", "franka")
namespace_camera = LaunchArgument("namespace_camera", "realsense")

TIMEOUT = 30.0


@dataclass
class Message:
    """Data class to define messages that will be obtained using the wait_for_message function.

    Attributes:
        topic (str): The ROS topic to subscribe to.
        msg_type (type): The type of the message.
        ros_value (None | Image | CameraInfo): The raw ROS message received.
        value (None | np.ndarray | data_utils.CameraInfo): The processed value.
    """

    topic: str
    msg_type: type
    ros_value: None | Image | CameraInfo = None
    value: None | np.ndarray | data_utils.CameraInfo = None

    def get_message(self, node: Node) -> bool:
        """Get the message from the topic.

        Args:
            node (Node): The ROS 2 node to use.

        Returns:
            bool: True if the message was received successfully, False otherwise.
        """
        success, self.ros_value = wait_for_message(
            self.msg_type, node, self.topic, time_to_wait=TIMEOUT
        )
        if not success:
            node.get_logger().error(f"Failed to get message from {self.topic}")
        return success


class GraspLogic(Node):
    """Node to handle the logic for generating grasps using GraspNet."""

    def __init__(self) -> None:
        """Initialize the GraspLogic node."""
        super().__init__("grasp_logic_node")
        self.declare_parameter("namespace_arm", "")
        self.declare_parameter("namespace_camera", "")
        namespace_arm = (
            self.get_parameter("namespace_arm").get_parameter_value().string_value
        )
        namespace_camera = (
            self.get_parameter("namespace_camera").get_parameter_value().string_value
        )

        self.color = Message(
            topic=f"{namespace_camera}/color/image_raw", msg_type=Image
        )
        self.depth = Message(
            topic=f"{namespace_camera}/depth/image_rect_raw_float", msg_type=Image
        )
        self.camera_info = Message(
            topic=f"{namespace_camera}/depth/camera_info", msg_type=CameraInfo
        )
        self.cb_group = ReentrantCallbackGroup()

        self.create_service(
            Trigger, "/grasp/trigger", self.callback, callback_group=self.cb_group
        )

        self.grasp_generation_client = self.create_client(
            GenerateGraspnetGrasp, "/graspnet/generate", callback_group=self.cb_group
        )

        self.define_goal_pose_client = self.create_client(
            DefineGoalPose, f"/{namespace_arm}/moveit_manager/define_goal_pose"
        )
        self.move_hand_to_pose_client = self.create_client(
            MoveHandToPose, f"/{namespace_arm}/moveit_manager/move_hand_to_pose"
        )

        self.marker_client = self.create_client(
            AddMarker, f"/{namespace_arm}/moveit_manager/add_marker"
        )
        self.open_gripper_client = self.create_client(
            Trigger, f"/{namespace_arm}/open_gripper", callback_group=self.cb_group
        )

        self.close_gripper_client = self.create_client(
            Trigger, f"/{namespace_arm}/close_gripper", callback_group=self.cb_group
        )

        self.move_to_configuration_client = self.create_client(
            MoveToConfiguration,
            f"/{namespace_arm}/moveit_manager/move_to_configuration",
        )

        for name, client in [
            ("graspnet/generate", self.grasp_generation_client),
            ("define_goal_pose", self.define_goal_pose_client),
            ("move_hand_to_pose", self.move_hand_to_pose_client),
            ("add_marker", self.marker_client),
            ("open_gripper", self.open_gripper_client),
            ("close_gripper", self.close_gripper_client),
            ("move_to_configuration", self.move_to_configuration_client),
        ]:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Service '/{name}' not available")
        ros_logger.info("GraspLogicNode initialized!")

    def get_messages(self) -> bool:
        """Get the messages from the topics and convert to correct format.

        Returns:
            bool: True if all messages are successfully retrieved, False otherwise.
        """
        for message in [self.color, self.depth, self.camera_info]:
            if not message.get_message(self):
                return False
        return True

    def _call(
        self, client: Client, request: Any, timeout: float, tag: str
    ) -> Optional[Any]:
        """Call a service and handle timeout/exception consistently.

        Args:
            client (Client): The service client to call.
            request (Any): The service request.
            timeout (float): The timeout in seconds.
            tag (str): A tag to identify the service in logs.

        Returns:
            Optional[Any]: The service response if successful, None otherwise.
        """
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future=future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error(f"{tag}: timed out after {timeout}s")
            return None
        exc = future.exception()
        if exc is not None:
            self.get_logger().exception(f"{tag}: raised: {exc}")
            return None
        return future.result()

    def callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Callback function for the Trigger service.

        Args:
            _request (Trigger.Request): The service request.
            response (Trigger.Response): The service response.

        Returns:
            Trigger.Response: The service response.
        """
        if not self.get_messages():
            response.success = False
            response.message = "Failed to get messages"
            return response

        ros_logger.info("Calling grasp generation service...")
        graspnet_request: GenerateGraspnetGrasp.Request = (
            GenerateGraspnetGrasp.Request()
        )
        graspnet_request.color = self.color.ros_value
        graspnet_request.depth = self.depth.ros_value
        graspnet_request.camera_info = self.camera_info.ros_value

        graspnet_response = self._call(
            self.grasp_generation_client, graspnet_request, TIMEOUT, "GraspNet"
        )
        if (
            graspnet_response is None
            or not graspnet_response.success
            or len(graspnet_response.grasps) == 0
        ):
            response.success = False
            response.message = "GraspNet failed or returned no grasps"
            return response

        best_grasp: Grasp = graspnet_response.grasps[0]
        ros_logger.info(
            f"Number of grasps received: {len(graspnet_response.grasps)} with best grasp: {best_grasp}"
        )
        ros_logger.info("Calling grasping movement service...")

        define_goal_pose = DefineGoalPose.Request()
        define_goal_pose.pose = best_grasp.pose

        add_marker = AddMarker.Request()
        add_marker.marker_pose = best_grasp.pose
        _ = self._call(self.marker_client, add_marker, 5.0, "AddMarker")

        _ = self._call(self.open_gripper_client, Trigger.Request(), 10.0, "OpenGripper")

        def_res = self._call(
            self.define_goal_pose_client, define_goal_pose, TIMEOUT, "DefineGoalPose"
        )
        if def_res is None or not def_res.success:
            response.success = False
            response.message = "DefineGoalPose failed"
            return response

        ros_logger.info(f"DefineGoalPose response: {def_res}")

        mh_res = self._call(
            self.move_hand_to_pose_client,
            MoveHandToPose.Request(),
            TIMEOUT,
            "MoveHandToPose",
        )
        if mh_res is None or not mh_res.success:
            response.success = False
            response.message = "MoveHandToPose failed"
            return response

        ros_logger.info(f"MoveHandToPose response: {mh_res}")

        _ = self._call(
            self.close_gripper_client, Trigger.Request(), 10.0, "CloseGripper"
        )

        home = MoveToConfiguration.Request()
        home.configuration = "home"
        _ = self._call(
            self.move_to_configuration_client,
            home,
            TIMEOUT,
            "MoveToConfiguration(home)",
        )

        response.success = True
        response.message = "Pick sequence executed"
        return response


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and spin it.

    Args:
        args (list | None): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = GraspLogic()
    spin_node(node)


if __name__ == "__main__":
    main()
