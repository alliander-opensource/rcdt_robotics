#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from franka_msgs.action import Grasp
from rcdt_utilities.launch_utils import spin_executor
from rclpy import logging
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

ros_logger = logging.get_logger(__name__)


class CloseGripper(Node):
    """Node to close the gripper using the Grasp action from franka_msgs."""

    def __init__(self) -> None:
        """Initialize the CloseGripper node."""
        super().__init__("close_gripper")
        self.declare_parameter("namespace", value="")
        namespace = self.get_namespace().strip("/")

        cbg_service = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger, "close_gripper", self.callback, callback_group=cbg_service
        )

        cbg_client = MutuallyExclusiveCallbackGroup()
        self.client = ActionClient(
            self, Grasp, f"/{namespace}/fr3_gripper/grasp", callback_group=cbg_client
        )

        self.create_goal()

    def create_goal(self) -> None:
        """Create the goal for the Grasp action.

        This method initializes the goal with default values for width, epsilon, force, and speed.
        """
        self.goal = Grasp.Goal()
        self.goal.width = 0.0
        self.goal.epsilon.inner = self.goal.epsilon.outer = 0.08
        self.goal.force = 100.0
        self.goal.speed = 0.03

    def callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Callback for the close_gripper service.

        Args:
            _request (Trigger.Request): The request for the service.
            response (Trigger.Response): The response to be filled.

        Returns:
            Trigger.Response: The response indicating success or failure of the gripper closing operation.
        """
        response.success = self.close_gripper()
        return response

    def close_gripper(self) -> bool:
        """Close the gripper using the Grasp action.

        Returns:
            bool: True if the gripper was closed successfully, False otherwise.
        """
        if not self.client.wait_for_server(timeout_sec=3):
            self.get_logger().error("Gripper grasp client not available.")
            return False

        result: Grasp.Impl.GetResultService.Response = self.client.send_goal(self.goal)
        if not result.result.success:
            self.get_logger().error("Closing gripper did not succeed.")
        return result.result.success


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and start the executor.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = CloseGripper()
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
