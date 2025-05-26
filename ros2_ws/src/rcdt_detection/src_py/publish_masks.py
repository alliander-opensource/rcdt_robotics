#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import rclpy
from rcdt_messages.srv import PublishMasks as Srv
from rcdt_utilities.cv_utils import cv2_image_to_ros_image, ros_image_to_cv2_image
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node
from sensor_msgs.msg import Image

ros_logger = logging.get_logger(__name__)


class PublishMasks(Node):
    """Node to publish combined masks to a specified topic.

    This node provides a service that takes a list of masks and a topic name,
    combines the masks by taking the maximum value across all masks, and publishes
    the combined mask to the specified topic.

    Attributes:
        node_name (str): The name of the node.
        service_name (str): The name of the service provided by this node.
    """

    def __init__(self) -> None:
        """Initialize the PublishMasks node."""
        super().__init__("publish_masks")
        self.create_service(Srv, "/publish_masks", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        """Callback function to handle the service request.

        Args:
            request (Srv.Request): The request containing the list of masks and the topic name.
            response (Srv.Response): The response to be filled with success status.

        Returns:
            Srv.Response: The response indicating whether the masks were published successfully.
        """
        publisher = self.create_publisher(Image, request.topic, 10)
        masks_cv2 = [ros_image_to_cv2_image(mask) for mask in request.masks]
        if len(masks_cv2) == 0:
            ros_logger.warning("Given input has zero masks. Cancel.")
            return response
        combined = np.max(masks_cv2, axis=0)
        publisher.publish(cv2_image_to_ros_image(combined))
        response.success = True
        return response


def main(args: str = None) -> None:
    """Main function to initialize the ROS 2 node and spin it.

    Args:
        args (str, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = PublishMasks()
    spin_node(node)


if __name__ == "__main__":
    main()
