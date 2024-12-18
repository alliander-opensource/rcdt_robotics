#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy import logging
from rclpy.node import Node
from rcdt_utilities.launch_utils import start_node
from rcdt_detection_msgs.srv import PublishImage
from sensor_msgs.msg import Image

ros_logger = logging.get_logger(__name__)


class PublishImageNode(Node):
    def __init__(self) -> None:
        super().__init__("publish_image")
        self.create_service(PublishImage, "/publish_image", self.callback)

    def callback(
        self, request: PublishImage.Request, response: PublishImage.Response
    ) -> PublishImage.Response:
        publisher = self.create_publisher(Image, request.topic, 10)
        publisher.publish(request.image)
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = PublishImageNode()
    start_node(node)


if __name__ == "__main__":
    main()
