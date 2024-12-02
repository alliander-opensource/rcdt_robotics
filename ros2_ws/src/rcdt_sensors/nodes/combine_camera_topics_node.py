#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy import logging
from rclpy.node import Node
from sensor_msgs.msg import Image
from realsense2_camera_msgs.msg import RGBD

ros_logger = logging.get_logger(__name__)


class CombineCameraTopicsNode(Node):
    def __init__(self) -> bool:
        super().__init__("combine_camera_topics")
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("d_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("rgbd_topic", "/camera/rgbd")

        rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        d_topic = self.get_parameter("d_topic").get_parameter_value().string_value
        rgbd_topic = self.get_parameter("rgbd_topic").get_parameter_value().string_value

        self.create_subscription(Image, rgb_topic, self.update_rgb, 10)
        self.create_subscription(Image, d_topic, self.update_d, 10)

        self.rgbd = RGBD()
        self.pub = self.create_publisher(RGBD, rgbd_topic, 10)

    def publish_rgbd(self) -> None:
        self.pub.publish(self.rgbd)

    def update_rgb(self, msg: Image) -> None:
        self.rgbd.rgb = msg
        self.publish_rgbd()

    def update_d(self, msg: Image) -> None:
        self.rgbd.depth = msg
        self.publish_rgbd()


def main(args: str = None) -> None:
    rclpy.init(args=args)
    try:
        node = CombineCameraTopicsNode()
        rclpy.spin(node)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
