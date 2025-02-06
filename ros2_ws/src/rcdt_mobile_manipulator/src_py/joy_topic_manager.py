#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from logging import getLogger

import mashumaro.codecs.yaml as yaml_codec
import rclpy
from rcdt_utilities.launch_utils import get_file_path, get_yaml, spin_executor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, Publisher
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

logger = getLogger(__name__)

@dataclass
class Output:
    button: int
    topic: str | None = None
    moveit: bool = False
    state: int | None = None

    def state_changed(self, state: bool) -> bool:
        if self.state in [None, state]:
            self.state = state
            return False
        else:
            self.state = state
            return True


class JoyTopicManager(Node):
    def __init__(self) -> bool:
        super().__init__("joy_topic_manager")
        self.declare_parameter("joy_topic", value="/joy")
        joy_input = self.get_parameter("joy_topic").get_parameter_value().string_value

        cbg = MutuallyExclusiveCallbackGroup()
        self.client = self.create_client(
            SetBool, "/servo_node/pause_servo", callback_group=cbg
        )

        file = get_file_path("rcdt_mobile_manipulator", ["config"], "joy_topics.yaml")
        yaml = get_yaml(file)

        self.outputs = [yaml_codec.decode(str(yaml[output]), Output) for output in yaml]
        self.pubs: dict[str, Publisher] = {}
        for output in self.outputs:
            if output.topic is not None:
                self.pubs[output.topic] = self.create_publisher(Joy, output.topic, 10)

        self.topic = None
        self.create_subscription(Joy, joy_input, self.handle_joy_message, 10)

    def handle_joy_message(self, msg: Joy) -> None:
        self.apply_output_changes(msg)
        self.pass_joy_message(msg)

    def apply_output_changes(self, msg: Joy) -> None:
        for output in self.outputs:
            state = msg.buttons[output.button]
            if output.state_changed(state) and self.topic_changed(output.topic):
                self.toggle_servo(pause=not output.moveit)

    def topic_changed(self, topic: str) -> bool:
        if self.topic == topic:
            return False
        self.topic = topic
        if self.topic is None:
            logger.info("Joy topic passing stopped.")
        else:
            logger.info(f"Joy topic is now passed to {self.topic}.")
        return True

    def toggle_servo(self, pause: bool) -> None:
        request = SetBool.Request()
        request.data = pause
        action = "pausing" if pause else "starting"
        logger.info(f"{action} moveit servo...")
        response: SetBool.Response = self.client.call(request)
        if response.success:
            logger.info(f"{action} moveit servo succeeded.")
        else:
            logger.info(f"{action} moveit servo failed.")

    def pass_joy_message(self, msg: Joy) -> None:
        if self.topic is None:
            return
        pub = self.pubs[self.topic]
        pub.publish(msg)


def main(args: str = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = JoyTopicManager()
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
