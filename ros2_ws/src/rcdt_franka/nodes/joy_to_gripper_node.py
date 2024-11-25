#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Joy
from franka_msgs.action import Move, Grasp
from rcdt_utilities.launch_utils import get_yaml, get_file_path


class Fr3GripperClient(Node):
    def __init__(self):
        super().__init__("fr3_gripper_client")
        self.move_client = ActionClient(self, Move, "/fr3_gripper/move")
        self.grasp_client = ActionClient(self, Grasp, "/fr3_gripper/grasp")
        self.is_gripped = False

    def open_gripper(self) -> None:
        if not self.is_gripped:
            return
        goal = Move.Goal()
        goal.width = 0.08
        goal.speed = 0.03

        if not self.move_client.wait_for_server(timeout_sec=2):
            self.get_logger().warn("Gripper move client not available.")
            return

        result: Move.Impl.GetResultService.Response = self.move_client.send_goal(goal)
        if not result.result.success:
            self.get_logger().warn("Opening gripper did not succeed.")
        self.is_gripped = False

    def close_gripper(self) -> None:
        if self.is_gripped:
            return
        goal = Grasp.Goal()
        goal.width = 0.0
        goal.epsilon.inner = goal.epsilon.outer = 0.08
        goal.force = 100.0
        goal.speed = 0.03

        if not self.grasp_client.wait_for_server(timeout_sec=2):
            self.get_logger().warn("Gripper grasp client not available.")
            return

        result: Grasp.Impl.GetResultService.Response = self.grasp_client.send_goal(goal)
        if not result.result.success:
            self.get_logger().warn("Closing gripper did not succeed.")
        self.is_gripped = True


class JoyToGripper(Node):
    def __init__(self, gripper_client: Fr3GripperClient):
        super().__init__("joy_to_gripper")
        self.gripper_client = gripper_client

        self.declare_parameter("sub_topic", value="/joy")
        sub_topic = self.get_parameter("sub_topic").get_parameter_value().string_value
        self.create_subscription(Joy, sub_topic, self.handle_input, 10)

        config = get_file_path("rcdt_franka", ["config"], "gamepad_mapping.yaml")
        self.mapping: dict = get_yaml(config)
        self.button_actions: dict = self.mapping.get("buttons", {})
        self.button_states = {}
        for button in self.button_actions:
            self.button_states[button] = None

    def handle_input(self, sub_msg: Joy) -> None:
        for button, action in self.button_actions.items():
            state = sub_msg.buttons[button]
            if self.button_states[button] is None:
                self.button_states[button] = state
            if state == self.button_states[button]:
                continue
            self.button_states[button] = state
            match action:
                case "open_gripper":
                    self.gripper_client.open_gripper()
                case "close_gripper":
                    self.gripper_client.close_gripper()


def main(args: str = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    gripper_client = Fr3GripperClient()
    executor.add_node(gripper_client)

    joy_to_gripper = JoyToGripper(gripper_client)
    executor.add_node(joy_to_gripper)

    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
