#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rcdt_messages.srv import GetRGBDFromTopic as Srv
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging, wait_for_message
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD

ros_logger = logging.get_logger(__name__)


class GetRGBDFromTopic(Node):
    """Node to get RGBD data from a specified topic.

    This node provides a service that waits for an RGBD message on a specified topic
    and returns the RGB image, depth image, and their respective camera info.

    Attributes:
        node_name (str): The name of the node.
        service_name (str): The name of the service provided by this node.
    """

    def __init__(self) -> None:
        """Node to get RGBD data from a specified topic."""
        super().__init__("get_rgbd_from_topic")
        self.create_service(Srv, "/get_rgbd_from_topic", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        """Callback function to handle the service request.

        Args:
            request (Srv.Request): The request containing the topic to listen to.
            response (Srv.Response): The response to be filled with RGBD data.

        Returns:
            Srv.Response: The response containing the RGB image, depth image, and camera info.
        """
        if not request.topic:
            ros_logger.error("No topic was specified. Exit.")
            response.success = False
            return response
        response.success, rgbd = wait_for_message.wait_for_message(
            RGBD, self, request.topic, time_to_wait=1
        )
        rgbd: RGBD
        if response.success:
            response.rgb_image = rgbd.rgb
            response.depth_image = rgbd.depth
            response.rgb_info = rgbd.rgb_camera_info
            response.depth_info = rgbd.depth_camera_info
        else:
            ros_logger.error(f"No RGBD message received on {request.topic}.")
        return response


def main(args: str = None) -> None:
    """Main function to initialize the ROS 2 node and spin it.

    Args:
        args (str, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = GetRGBDFromTopic()
    spin_node(node)


if __name__ == "__main__":
    main()
