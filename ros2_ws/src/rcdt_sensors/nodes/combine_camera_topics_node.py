#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from realsense2_camera_msgs.msg import RGBD


class JoyToTwistNode(Node):
    def __init__(self) -> bool:
        super().__init__("combine_camera_topics")
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("d_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("rgbd_topic", "/camera/rgbd")
        self.declare_parameter("rate", 30)

        rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        d_topic = self.get_parameter("d_topic").get_parameter_value().string_value
        rgbd_topic = self.get_parameter("rgbd_topic").get_parameter_value().string_value
        rate = self.get_parameter("rate").get_parameter_value().integer_value

        self.create_subscription(Image, rgb_topic, self.update_rgb, 10)
        self.create_subscription(Image, d_topic, self.update_d, 10)
        self.update = False

        self.rgbd = RGBD()
        self.pub = self.create_publisher(RGBD, rgbd_topic, 10)
        self.create_timer(1 / rate, self.publish_rgbd)
        self.run()

    def publish_rgbd(self) -> None:
        if self.update:
            self.pub.publish(self.rgbd)
            self.update = False

    def update_rgb(self, msg: Image) -> None:
        self.rgbd.rgb = msg
        self.update = True

    def update_d(self, msg: Image) -> None:
        self.rgbd.depth = msg
        self.update = False

    def run(self) -> None:
        rclpy.spin(self)


def main(args: str = None) -> None:
    rclpy.init(args=args)
    JoyToTwistNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
