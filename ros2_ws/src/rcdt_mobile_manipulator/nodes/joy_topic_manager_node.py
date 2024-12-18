#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List
import rclpy
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Joy
from rcdt_utilities.launch_utils import start_node
from rcdt_utilities.launch_utils import get_yaml, get_file_path


class JoyTopicManager(Node):
    def __init__(self) -> bool:
        super().__init__("joy_topic_manager")
        self.declare_parameter("joy_topic", value="/joy")
        joy_topic = self.get_parameter("joy_topic").get_parameter_value().string_value

        file = get_file_path("rcdt_mobile_manipulator", ["config"], "joy_topics.yaml")
        configs: dict = get_yaml(file)
        self.topics: List[str] = []
        self.pubs: dict[str, Publisher] = {}
        self.buttons: List[int] = []
        self.button_states: List[int] = []

        for config in configs.values():
            topic = config.get("topic")
            button = config.get("button")
            self.topics.append(topic)
            self.pubs[topic] = (
                None if topic == "" else self.create_publisher(Joy, topic, 10)
            )
            self.buttons.append(button)
            self.button_states.append(None)

        self.topic = ""
        self.create_subscription(Joy, joy_topic, self.handle_joy_message, 10)

    def handle_joy_message(self, msg: Joy) -> None:
        self.update_publish_topic(msg)
        self.pass_joy_message(msg)

    def update_publish_topic(self, msg: Joy) -> None:
        for idx in range(len(self.buttons)):
            button = self.buttons[idx]
            button_state = msg.buttons[button]
            if self.button_states[idx] is None:
                self.button_states[idx] = button_state
            if button_state == self.button_states[idx]:
                continue
            self.button_states[idx] = button_state
            topic = self.topics[idx]
            if self.topic == topic:
                continue
            self.topic = topic
            if self.topic == "":
                self.get_logger().info("Joy topic passing stopped.")
            else:
                self.get_logger().info(f"Joy topic is now passed to {self.topic}.")

    def pass_joy_message(self, msg: Joy) -> None:
        if self.topic == "":
            return
        pub = self.pubs[self.topic]
        pub.publish(msg)


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = JoyTopicManager()
    start_node(node)


if __name__ == "__main__":
    main()
