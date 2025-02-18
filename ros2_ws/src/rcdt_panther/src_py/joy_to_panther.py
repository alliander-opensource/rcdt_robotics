#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node
from sensor_msgs.msg import Joy

ros_logger = logging.get_logger(__name__)


class JoyToPanther(Node):
    def __init__(self) -> bool:
        super().__init__("joy_to_twist_node")
        self.create_subscription(Joy, "/joy", self.handle_input, 10)
        self.pub = self.create_publisher(Joy, "/panther/joy", 10)

    def handle_input(self, msg: Joy) -> None:
        right = msg.axes.pop(5)
        left = msg.axes.pop(2)

        buttons = [0] * 12
        buttons[0] = msg.buttons[2]
        buttons[1] = msg.buttons[0]
        buttons[2] = msg.buttons[1]
        buttons[3] = msg.buttons[3]
        buttons[4] = msg.buttons[4]
        buttons[5] = msg.buttons[5]
        buttons[6] = 1 if left == -1 else 0
        buttons[7] = 1 if right == -1 else 0

        msg.buttons = buttons
        self.pub.publish(msg)


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = JoyToPanther()
    spin_node(node)


if __name__ == "__main__":
    main()
