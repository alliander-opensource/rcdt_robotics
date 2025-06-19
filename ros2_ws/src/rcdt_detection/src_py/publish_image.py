#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rcdt_messages.srv import PublishImage as Srv
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node
from sensor_msgs.msg import Image

ros_logger = logging.get_logger(__name__)


class PublishImage(Node):
    """Node to publish an image to a specified topic.

    This node provides a service that takes an image and a topic name,
    and publishes the image to that topic.
    """

    def __init__(self) -> None:
        """Initialize the PublishImage node."""
        super().__init__("publish_image")
        self.create_service(Srv, "/publish_image", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        """Callback function to handle the service request.

        Args:
            request (Srv.Request): The request containing the image and topic name.
            response (Srv.Response): The response to be filled with success status.

        Returns:
            Srv.Response: The response indicating whether the image was published successfully.
        """
        publisher = self.create_publisher(Image, request.topic, 10)
        publisher.publish(request.image)
        response.success = True
        return response


def main(args: str = None) -> None:
    """Main function to initialize the ROS 2 node and spin it.

    Args:
        args (str, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = PublishImage()
    spin_node(node)


if __name__ == "__main__":
    main()
