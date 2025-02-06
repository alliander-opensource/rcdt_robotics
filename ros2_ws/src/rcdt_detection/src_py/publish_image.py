#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from rcdt_utilities.launch_utils import spin_node
from rcdt_detection_msgs.srv import PublishImage as Srv
from sensor_msgs.msg import Image


class PublishImage(Node):
    def __init__(self) -> None:
        super().__init__("publish_image")
        self.create_service(Srv, "/publish_image", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        publisher = self.create_publisher(Image, request.topic, 10)
        publisher.publish(request.image)
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = PublishImage()
    spin_node(node)


if __name__ == "__main__":
    main()
