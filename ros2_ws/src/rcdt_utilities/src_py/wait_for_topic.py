#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

MSG_TYPES = {"JointState": JointState, "Clock": Clock, "Empty": Empty}
WAIT = 3


class WaitForTopic(Node):
    def __init__(self) -> None:
        super().__init__("wait_for_topic")
        self.declare_parameter("topic", "")
        self.declare_parameter("msg_type", "")
        self.declare_parameter("log", True)
        log = self.get_parameter("log").get_parameter_value().bool_value

        topic = self.get_parameter("topic").get_parameter_value().string_value
        if log:
            self.get_logger().info(f"waiting for topic: {topic}")
        if topic == "":
            self.error("Empty topic is not supported.")

        msg_type_str = self.get_parameter("msg_type").get_parameter_value().string_value
        msg_type = MSG_TYPES.get(msg_type_str)
        if msg_type is None:
            self.error(f"msg_type '{msg_type_str}' is not supported.")

        success = False
        while not success:
            try:
                success, _ = wait_for_message(msg_type, self, topic, time_to_wait=WAIT)
            except KeyboardInterrupt:
                break
            if not success and log:
                self.get_logger().warn(
                    f"No message received on topic '{topic}' of type '{msg_type_str}'. Continue waiting..."
                )

    def error(self, msg: str) -> None:
        self.get_logger().error(msg)
        raise ValueError(msg)


def main(args: str = None) -> None:
    rclpy.init(args=args)
    WaitForTopic()


if __name__ == "__main__":
    main()
