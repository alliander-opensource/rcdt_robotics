#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from threading import Thread
from typing import Literal

import rclpy
from rcdt_utilities.launch_utils import get_file_path, get_yaml, spin_executor
from rclpy import logging
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

ros_logger = logging.get_logger(__name__)


class JoyToGripper(Node):
    def __init__(self):
        super().__init__("joy_to_gripper")

        cbg_open_gripper = MutuallyExclusiveCallbackGroup()
        self.open_gripper = self.create_client(
            Trigger, "open_gripper", callback_group=cbg_open_gripper
        )

        cbg_close_gripper = MutuallyExclusiveCallbackGroup()
        self.close_gripper = self.create_client(
            Trigger, "close_gripper", callback_group=cbg_close_gripper
        )

        self.declare_parameter("sub_topic", value="/joy")
        sub_topic = self.get_parameter("sub_topic").get_parameter_value().string_value
        self.create_subscription(Joy, sub_topic, self.handle_input, 10)

        config = get_file_path("rcdt_franka", ["config"], "gamepad_mapping.yaml")
        self.mapping: dict = get_yaml(config)
        self.button_actions: dict = self.mapping.get("buttons", {})
        self.button_states = {}
        for button in self.button_actions:
            self.button_states[button] = None

        self.busy = False

    def handle_input(self, sub_msg: Joy) -> None:
        for button, action in self.button_actions.items():
            state = sub_msg.buttons[button]
            if self.button_states[button] is None:
                self.button_states[button] = state
            if state == self.button_states[button]:
                continue
            self.button_states[button] = state
            thread = Thread(target=self.perform_action_if_not_busy, args=[action])
            thread.start()

    def perform_action_if_not_busy(
        self, action: Literal["open_gripper", "close_gripper"]
    ) -> None:
        if self.busy:
            return
        self.busy = True
        match action:
            case "open_gripper":
                self.open_gripper.call(Trigger.Request())
            case "close_gripper":
                self.close_gripper.call(Trigger.Request())
        self.busy = False


def main(args: str = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = JoyToGripper()
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
