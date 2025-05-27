#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from importlib import reload

import rclpy
from rcdt_actions.actions import actions
from rcdt_messages.action import Sequence
from rcdt_utilities.launch_utils import spin_executor
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class ActionExecutor(Node):
    """ActionExecutor node to handle action requests for sequences."""

    def __init__(self) -> bool:
        """Initialize the ActionExecutor node."""
        super().__init__("action_executor")

        self.action_server = ActionServer(
            self,
            Sequence,
            "/action_executor",
            self.callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def callback(self, goal_handle: ServerGoalHandle) -> Sequence.Result:
        """Callback for the action server.

        This method is called when a goal is received by the action server.
        It checks if the requested sequence exists, executes it, and returns the result.

        Args:
            goal_handle (ServerGoalHandle): The handle for the goal.

        Returns:
            Sequence.Result: The result of the sequence execution.
        """
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
    """Main function to initialize the ROS 2 node and start the action executor."""
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ActionExecutor()
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
