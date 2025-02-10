#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node
from realsense2_camera_msgs.msg import RGBD
from sensor_msgs.msg import CameraInfo, Image

ros_logger = logging.get_logger(__name__)


class CombineCameraTopics(Node):
    def __init__(self) -> bool:
        super().__init__("combine_camera_topics")
        self.declare_parameter("rgb", "/camera/camera/color/image_raw")
        self.declare_parameter("depth", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("rgb_info", "/camera/camera/color/camera_info")
        self.declare_parameter("depth_info", "/camera/camera/depth/camera_info")
        self.declare_parameter("rgbd", "/camera/camera/rgbd")

        rgb = self.get_parameter("rgb").get_parameter_value().string_value
        depth = self.get_parameter("depth").get_parameter_value().string_value
        rgb_info = self.get_parameter("rgb_info").get_parameter_value().string_value
        depth_info = self.get_parameter("depth_info").get_parameter_value().string_value
        rgbd = self.get_parameter("rgbd").get_parameter_value().string_value

        self.create_subscription(Image, rgb, self.update_rgb, 10)
        self.create_subscription(Image, depth, self.update_depth, 10)
        self.create_subscription(CameraInfo, rgb_info, self.update_rgb_info, 10)
        self.create_subscription(CameraInfo, depth_info, self.update_depth_info, 10)

        self.rgbd = RGBD()
        self.pub = self.create_publisher(RGBD, rgbd, 10)
        self.create_timer(1 / 30, self.publish_rgbd)

    def publish_rgbd(self) -> None:
        self.pub.publish(self.rgbd)

    def update_rgb(self, msg: Image) -> None:
        self.rgbd.rgb = msg

    def update_depth(self, msg: Image) -> None:
        self.rgbd.depth = msg

    def update_rgb_info(self, msg: CameraInfo) -> None:
        self.rgbd.rgb_camera_info = msg

    def update_depth_info(self, msg: CameraInfo) -> None:
        self.rgbd.depth_camera_info = msg


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = CombineCameraTopics()
    spin_node(node)


if __name__ == "__main__":
    main()
