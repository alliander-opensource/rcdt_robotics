#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy import logging
from rclpy.node import Node
from rcdt_detection_msgs.srv import SplitRGBD
from rcdt_utilities.launch_utils import start_node

ros_logger = logging.get_logger(__name__)


class SplitRGBDNode(Node):
    def __init__(self) -> None:
        super().__init__("split_rgbd")
        self.create_service(SplitRGBD, "/split_rgbd", self.callback)

    def callback(
        self, request: SplitRGBD.Request, response: SplitRGBD.Response
    ) -> SplitRGBD.Response:
        response.rgb_image = request.rgbd.rgb
        response.depth_image = request.rgbd.depth
        response.rgb_info = request.rgbd.rgb_camera_info
        response.depth_info = request.rgbd.depth_camera_info
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = SplitRGBDNode()
    start_node(node)


if __name__ == "__main__":
    main()
