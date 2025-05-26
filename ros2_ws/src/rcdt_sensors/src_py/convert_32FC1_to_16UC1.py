#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import rclpy
from rcdt_utilities.cv_utils import cv2_image_to_ros_image, ros_image_to_cv2_image
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node
from sensor_msgs.msg import Image

ros_logger = logging.get_logger(__name__)


class Convert32FC1to16UC1(Node):
    """A ROS2 node that converts 32FC1 depth images to 16UC1 format.

    This node subscribes to a 32FC1 depth image topic, converts the image data to
    16UC1 format (millimeters), and publishes the converted image to a new topic.

    Attributes:
        publisher (Publisher): Publisher for the converted 16UC1 depth image.
    """

    def __init__(self) -> bool:
        """Initialize the Convert32FC1to16UC1 node."""
        super().__init__("convert_32FC1_to_16UC1_node")

        self.create_subscription(
            Image, "/camera/camera/depth/image_rect_raw_float", self.callback, 10
        )
        self.publisher = self.create_publisher(
            Image, "/camera/camera/depth/image_rect_raw", 10
        )

    def callback(self, msg: Image) -> None:
        """Callback function to process incoming 32FC1 depth images.

        Args:
            msg (Image): The incoming 32FC1 depth image message.
        """
        cv2_image = ros_image_to_cv2_image(msg)
        cv2_image *= 1000  # m to mm
        cv2_image = cv2_image.astype(np.uint16)
        ros_image = cv2_image_to_ros_image(cv2_image)
        ros_image.header = msg.header
        self.publisher.publish(ros_image)


def main(args: str = None) -> None:
    """Main function to initialize the ROS2 node and spin it.

    Args:
        args (str, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = Convert32FC1to16UC1()
    spin_node(node)


if __name__ == "__main__":
    main()
