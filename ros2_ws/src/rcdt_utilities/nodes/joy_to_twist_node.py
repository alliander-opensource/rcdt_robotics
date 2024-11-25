#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from rcdt_utilities.launch_utils import get_yaml, get_file_path


class JoyToTwistNode(Node):
    def __init__(self) -> bool:
        super().__init__("joy_to_twist_node")
        self.declare_parameter("sub_topic", "/joy")
        self.declare_parameter("pub_topic", "")
        self.declare_parameter("config_pkg", "")
        self.declare_parameter("pub_frame", "")

        sub_topic = self.get_parameter("sub_topic").get_parameter_value().string_value
        pub_topic = self.get_parameter("pub_topic").get_parameter_value().string_value
        config_pkg = self.get_parameter("config_pkg").get_parameter_value().string_value
        pub_frame = self.get_parameter("pub_frame").get_parameter_value().string_value

        if sub_topic == "":
            self.get_logger().warn("No subscriber topic was specified. Exiting.")
            return

        if pub_topic == "":
            self.get_logger().warn("No publisher topic was specified. Exiting.")
            return

        if config_pkg == "":
            self.get_logger().warn("No package for config file was specified. Exiting.")
            return

        config = get_file_path(config_pkg, ["config"], "gamepad_mapping.yaml")
        self.mapping = get_yaml(config)

        if self.mapping == "":
            self.get_logger().warn(f"No mapping was found in {config}. Exiting.")
            return

        self.create_subscription(Joy, sub_topic, self.handle_input, 10)
        self.pub = self.create_publisher(TwistStamped, pub_topic, 10)
        self.pub_msg = TwistStamped()
        self.pub_msg.header.frame_id = pub_frame
        self.profile = "A"
        self.run()

    def run(self) -> None:
        rclpy.spin(self)

    def handle_input(self, sub_msg: Joy) -> None:
        for idx in range(len(sub_msg.axes)):
            value = sub_msg.axes[idx]
            if idx not in self.mapping["axes"]:
                continue
            axis: dict = self.mapping["axes"][idx]

            if axis == "switch_profile":
                self.profile = "A" if value > -1 else "B"
                continue

            for movement in ["angular", "linear"]:
                config: dict = axis.get(movement)
                if config is None:
                    continue
                profile = config.get("profile", self.profile)
                if profile != self.profile:
                    continue
                value *= -1 if config.get("flip", False) else 1
                direction = config["direction"]
                vector = getattr(self.pub_msg.twist, movement)
                setattr(vector, direction, value)
        self.pub_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.pub_msg)


def main(args: str = None) -> None:
    rclpy.init(args=args)
    JoyToTwistNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
