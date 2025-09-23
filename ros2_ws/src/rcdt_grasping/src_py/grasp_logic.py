#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass

import numpy as np
import rclpy
from graspnetpy_utils import data_utils
from rcdt_messages.srv import GenerateGraspnetGrasp
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

ros_logger = logging.get_logger(__name__)

TIMEOUT = 5.0


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

        self.color = Message(topic="/franka/realsense/color/image_raw", msg_type=Image)
        self.depth = Message(
            topic="/franka/realsense/depth/image_rect_raw_float", msg_type=Image
        )
        self.camera_info = Message(
            topic="/franka/realsense/depth/camera_info", msg_type=CameraInfo
        )
        self.cb_group = ReentrantCallbackGroup()

        self.create_service(
            Trigger, "/grasp/trigger", self.callback, callback_group=self.cb_group
        )

        self.client = self.create_client(
            GenerateGraspnetGrasp, "/graspnet/generate", callback_group=self.cb_group
        )

        self.client.wait_for_service(timeout_sec=TIMEOUT)
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

    def callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Callback function for the Trigger service.

        Args:
            request (Trigger.Request): The service request.
            response (Trigger.Response): The service response.

        Returns:
            Trigger.Response: The service response.
        """
        if not self.get_messages():
            response.success = False
            response.message = "Failed to get messages"
            return response

        request: GenerateGraspnetGrasp.Request = GenerateGraspnetGrasp.Request()
        request.color = self.color.ros_value
        request.depth = self.depth.ros_value
        request.camera_info = self.camera_info.ros_value

        ros_logger.info("Calling grasp generation service...")
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future=future, timeout_sec=30.0)

        if future.result().success is not True:
            ros_logger.error(f"Service call failed: {future.exception()}")
            response.success = False
            response.message = "Service call failed"
            return response

        ros_logger.info(f"Service call succeeded: {future.result().success}")

        response.success = True
        response.message = "Messages received successfully"
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
