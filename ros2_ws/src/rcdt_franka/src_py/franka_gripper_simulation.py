#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from control_msgs.action import ParallelGripperCommand
from franka_msgs.action import Grasp, Homing, Move
from rcdt_utilities.launch_utils import spin_executor
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

MAX = 0.039
MIN = 0.001


class GripperActionControllerClient(Node):
    """Client for the Gripper Action Controller."""

    def __init__(self):
        """Initialize the Gripper Action Controller Client."""
        super().__init__("gripper_action_controller_client")
        self.client = ActionClient(
            self,
            ParallelGripperCommand,
            "/franka/gripper_action_controller/gripper_cmd",
        )

    def move(self, width: float) -> bool:
        """Move the gripper to a specified width.

        Args:
            width (float): The desired width to move the gripper to.

        Returns:
            bool: True if the gripper reached the goal, False otherwise.
        """
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [self.respect_limits(width)]
        self.client.wait_for_server()
        result = self.client.send_goal(goal)
        result: ParallelGripperCommand.Impl.GetResultService.Response
        return result.result.reached_goal

    def grasp(self, width: float, max_effort: float) -> bool:
        """Grasp with the gripper at a specified width and maximum effort.

        Args:
            width (float): The desired width to grasp.
            max_effort (float): The maximum effort to apply during the grasp.

        Returns:
            bool: True if the gripper reached the goal or stalled, False otherwise.
        """
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [self.respect_limits(width)]
        goal.command.effort = [max_effort]
        self.client.wait_for_server()
        result = self.client.send_goal(goal)
        result: ParallelGripperCommand.Impl.GetResultService.Response
        return result.result.reached_goal or result.result.stalled

    @staticmethod
    def respect_limits(width: float) -> float:
        """Ensure the gripper width is within the defined limits.

        Args:
            width (float): The desired width to set for the gripper.

        Returns:
            float: The width adjusted to be within the limits defined by MIN and MAX.
        """
        return min(MAX, max(MIN, width))


class FrankaGripperSimulation(Node):
    """Node to simulate the Franka Gripper using action servers."""

    def __init__(self, gripper_action_controller_client: GripperActionControllerClient):
        """Initialize the Franka Gripper Simulation node.

        Args:
            gripper_action_controller_client (GripperActionControllerClient): The client to interact with the gripper action controller.
        """
        super().__init__("fr3_gripper")
        self.gripper_action_client = gripper_action_controller_client
        ActionServer(self, Grasp, "~/grasp", self.grasp_action)
        ActionServer(self, Homing, "~/homing", self.homing_action)
        ActionServer(self, Move, "~/move", self.move_action)

    def grasp_action(self, goal_handle: ServerGoalHandle) -> Grasp.Result:
        """Handle the Grasp action request.

        Args:
            goal_handle (ServerGoalHandle): The handle for the goal.

        Returns:
            Grasp.Result: The result of the grasp action, indicating success or failure.
        """
        self.get_logger().info("Gripper Grasping...")
        request: Grasp.Goal = goal_handle.request
        result = Grasp.Result()
        if self.gripper_action_client.grasp(0, request.force):
            goal_handle.succeed()
            result.success = True
            self.get_logger().info("Gripper Grasping succeeded")
        return result

    def homing_action(self, goal_handle: ServerGoalHandle) -> Homing.Result:
        """Handle the Homing action request.

        Args:
            goal_handle (ServerGoalHandle): The handle for the goal.

        Returns:
            Homing.Result: The result of the homing action, indicating success or failure.
        """
        self.get_logger().info("Gripper Homing...")
        result = Homing.Result()
        if self.gripper_action_client.move(MAX):
            goal_handle.succeed()
            result.success = True
            self.get_logger().info("Gripper Homing succeeded")
        return result

    def move_action(self, goal_handle: ServerGoalHandle) -> Move.Result:
        """Handle the Move action request.

        Args:
            goal_handle (ServerGoalHandle): The handle for the goal.

        Returns:
            Move.Result: The result of the move action, indicating success or failure.
        """
        self.get_logger().info("Gripper Moving...")
        request: Move.Goal = goal_handle.request
        result = Move.Result()
        if self.gripper_action_client.move(request.width):
            goal_handle.succeed()
            result.success = True
            self.get_logger().info("Gripper Moving succeeded")
        return result


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and start the executor.

    Args:
        args (list | None, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    gripper_action_controller_client = GripperActionControllerClient()
    executor.add_node(gripper_action_controller_client)

    fr3_gripper = FrankaGripperSimulation(gripper_action_controller_client)
    executor.add_node(fr3_gripper)

    spin_executor(executor)


if __name__ == "__main__":
    main()
