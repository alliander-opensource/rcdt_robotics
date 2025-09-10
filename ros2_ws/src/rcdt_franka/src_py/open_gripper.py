#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from franka_msgs.action import Move
from rcdt_utilities.launch_utils import spin_executor
from rclpy import logging
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

ros_logger = logging.get_logger(__name__)


class OpenGripper(Node):
    """Node to open the gripper using the Move action from franka_msgs."""

    def __init__(self) -> None:
        """Initialize the OpenGripper node."""
        super().__init__("open_gripper")

        cbg_service = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger, "open_gripper", self.callback, callback_group=cbg_service
        )

        cbg_client = MutuallyExclusiveCallbackGroup()
        self.client = ActionClient(
            self, Move, "/franka/fr3_gripper/move", callback_group=cbg_client
        )

        self.create_goal()

    def create_goal(self) -> None:
        """Create the goal for the Move action."""
        self.goal = Move.Goal()
        self.goal.width = 0.08
        self.goal.speed = 0.03

    def callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Callback for the open_gripper service.

        Args:
            _request (Trigger.Request): The request for the service.
            response (Trigger.Response): The response to be filled.

        Returns:
            Trigger.Response: The response indicating success or failure of the gripper opening operation.
        """
        response.success = self.open_gripper()
        return response

    def open_gripper(self) -> bool:
        """Open the gripper using the Move action.

        This method sends a goal to the gripper action server to open the gripper.

        Returns:
            bool: True if the gripper opened successfully, False otherwise.
        """
        if not self.client.wait_for_server(timeout_sec=3):
            self.get_logger().error("Gripper move client not available.")
            return False
        self.get_logger().info("Sending goal to open gripper service...")

        result: Move.Impl.GetResultService.Response = self.client.send_goal(self.goal)
        if not result.result.success:
            self.get_logger().error("Opening gripper did not succeed.")
        else:
            self.get_logger().info("Gripper opened successfully.")
        return result.result.success


def main(args: list | None = None) -> None:
    """Main function to run the OpenGripper node.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = OpenGripper()
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
