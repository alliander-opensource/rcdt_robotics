#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node

ros_logger = logging.get_logger(__name__)


class TwistToTwistStamped(Node):
    def __init__(self) -> bool:
        super().__init__("twist_to_twist_stamped_node")
        self.declare_parameter("sub_topic", "/coll_mntr_cmd_vel")
        self.declare_parameter("pub_topic", "/diff_drive_controller/cmd_vel")

        sub_topic = self.get_parameter("sub_topic").get_parameter_value().string_value
        pub_topic = self.get_parameter("pub_topic").get_parameter_value().string_value

        self.create_subscription(Twist, sub_topic, self.handle_input, 10)
        self.pub = self.create_publisher(TwistStamped, pub_topic, 10)
        self.pub_msg = TwistStamped()

    def handle_input(self, sub_msg: Twist) -> None:
        self.pub_msg.twist = sub_msg
        self.pub_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.pub_msg)


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    spin_node(node)


if __name__ == "__main__":
    main()
