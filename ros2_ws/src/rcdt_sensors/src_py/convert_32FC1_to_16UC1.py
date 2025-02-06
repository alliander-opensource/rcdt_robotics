#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import rclpy
from rcdt_utilities.cv_utils import cv2_image_to_ros_image, ros_image_to_cv2_image
from rcdt_utilities.launch_utils import spin_node
from rclpy.node import Node
from sensor_msgs.msg import Image


class Convert32FC1to16UC1(Node):
    def __init__(self) -> bool:
        super().__init__("convert_32FC1_to_16UC1_node")

        self.create_subscription(
            Image, "/camera/camera/depth/image_rect_raw_float", self.callback, 10
        )
        self.publisher = self.create_publisher(
            Image, "/camera/camera/depth/image_rect_raw", 10
        )

    def callback(self, msg: Image) -> None:
        cv2_image = ros_image_to_cv2_image(msg)
        cv2_image *= 1000  # m to mm
        cv2_image = cv2_image.astype(np.uint16)
        ros_image = cv2_image_to_ros_image(cv2_image)
        self.publisher.publish(ros_image)


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = Convert32FC1to16UC1()
    spin_node(node)


if __name__ == "__main__":
    main()
