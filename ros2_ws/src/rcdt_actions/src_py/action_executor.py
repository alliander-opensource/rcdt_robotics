#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from importlib import reload
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcdt_utilities.launch_utils import spin_executor
from rcdt_actions.actions import actions
from rcdt_messages.action import Sequence


class ActionExecutor(Node):
    def __init__(self) -> bool:
        super().__init__("action_executor")

        self.action_server = ActionServer(
            self,
            Sequence,
            "/action_executor",
            self.callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def callback(self, goal_handle: ServerGoalHandle) -> Sequence.Result:
        reload(actions)

        goal: Sequence.Goal = goal_handle.request
        result = Sequence.Result()

        if not hasattr(actions, goal.sequence):
            message = f"Sequence '{goal.sequence}' does not exist."
            self.get_logger().error(message)
            goal_handle.abort()
            result.success = False
            result.message = message
            return result

        sequence: actions.Sequence = getattr(actions, goal.sequence)
        sequence.node = self
        sequence.goal_handle = goal_handle
        sequence.execute()

        if sequence.success:
            goal_handle.succeed()
            result.message = f"Sequence '{goal.sequence}' was executed successfully."
        else:
            goal_handle.abort()
            result.message = f"Sequence '{goal.sequence}' was aborted."
        result.success = sequence.success
        return result


def main(args: str = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ActionExecutor()
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
