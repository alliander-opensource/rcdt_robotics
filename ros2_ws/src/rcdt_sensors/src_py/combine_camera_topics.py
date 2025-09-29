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
    """A ROS2 node that combines RGB and depth camera topics into a single RGBD message.

    This node subscribes to the RGB and depth image topics, as well as their corresponding
    camera info topics. It publishes a combined RGBD message that contains both the RGB image
    and the depth image, along with their camera information.
    """

    def __init__(self) -> None:
        """Initialize the CombineCameraTopics node."""
        super().__init__("combine_camera_topics")
        namespace = self.get_namespace().strip("/")

        self.declare_parameter("rgb", f"/{namespace}/color/image_raw")
        self.declare_parameter("depth", f"/{namespace}/depth/image_rect_raw")
        self.declare_parameter("rgb_info", f"/{namespace}/color/camera_info")
        self.declare_parameter("depth_info", f"/{namespace}/depth/camera_info")
        self.declare_parameter("rgbd", f"/{namespace}/rgbd")

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
        """Publish the combined RGBD message."""
        self.pub.publish(self.rgbd)

    def update_rgb(self, msg: Image) -> None:
        """Update the RGB image in the RGBD message.

        Args:
            msg (Image): The incoming RGB image message.
        """
        self.rgbd.rgb = msg

    def update_depth(self, msg: Image) -> None:
        """Update the depth image in the RGBD message.

        Args:
            msg (Image): The incoming depth image message.
        """
        self.rgbd.depth = msg

    def update_rgb_info(self, msg: CameraInfo) -> None:
        """Update the RGB camera info in the RGBD message.

        Args:
            msg (CameraInfo): The incoming RGB camera info message.
        """
        self.rgbd.rgb_camera_info = msg

    def update_depth_info(self, msg: CameraInfo) -> None:
        """Update the depth camera info in the RGBD message.

        Args:
            msg (CameraInfo): The incoming depth camera info message.
        """
        self.rgbd.depth_camera_info = msg


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and start spinning.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    node = CombineCameraTopics()
    spin_node(node)


if __name__ == "__main__":
    main()
