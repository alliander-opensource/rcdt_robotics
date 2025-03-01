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
    def __init__(self) -> None:
        super().__init__("get_rgbd_from_topic")
        self.create_service(Srv, "/get_rgbd_from_topic", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        if request.topic == "":
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
    rclpy.init(args=args)
    node = GetRGBDFromTopic()
    spin_node(node)


if __name__ == "__main__":
    main()
