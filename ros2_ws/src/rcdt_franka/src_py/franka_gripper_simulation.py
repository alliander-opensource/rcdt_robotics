#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from logging import getLogger

import rclpy
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp, Homing, Move
from rcdt_utilities.launch_utils import spin_executor
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

logger = getLogger(__name__)

MAX = 0.039
MIN = 0.001



class GripperActionControllerClient(Node):
    def __init__(self):
        super().__init__("gripper_action_controller_client")
        self.client = ActionClient(
            self, GripperCommand, "/gripper_action_controller/gripper_cmd"
        )

    def move(self, width: float) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = self.respect_limits(width)
        self.client.wait_for_server()
        result = self.client.send_goal(goal)
        result: GripperCommand.Impl.GetResultService.Response
        return result.result.reached_goal

    def grasp(self, width: float, max_effort: float) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = self.respect_limits(width)
        goal.command.max_effort = max_effort
        self.client.wait_for_server()
        result = self.client.send_goal(goal)
        result: GripperCommand.Impl.GetResultService.Response
        return result.result.reached_goal or result.result.stalled

    def respect_limits(self, width: float) -> float:
        return min(MAX, max(MIN, width))


class FrankaGripperSimulation(Node):
    def __init__(self, gripper_action_controller_client: GripperActionControllerClient):
        super().__init__("fr3_gripper")
        self.gripper_action_client = gripper_action_controller_client
        ActionServer(self, Grasp, "~/grasp", self.grasp_action)
        ActionServer(self, Homing, "~/homing", self.homing_action)
        ActionServer(self, Move, "~/move", self.move_action)

    def grasp_action(self, goal_handle: ServerGoalHandle) -> Grasp.Result:
        logger.info("Gripper Grasping...")
        request: Grasp.Goal = goal_handle.request
        result = Grasp.Result()
        if self.gripper_action_client.grasp(0, request.force):
            goal_handle.succeed()
            result.success = True
            logger.info("Gripper Grasping succeeded")
        return result

    def homing_action(self, goal_handle: ServerGoalHandle) -> Homing.Result:
        logger.info("Gripper Homing...")
        result = Homing.Result()
        if self.gripper_action_client.move(MAX):
            goal_handle.succeed()
            result.success = True
            logger.info("Gripper Homing succeeded")
        return result

    def move_action(self, goal_handle: ServerGoalHandle) -> Move.Result:
        logger.info("Gripper Moving...")
        request: Move.Goal = goal_handle.request
        result = Move.Result()
        if self.gripper_action_client.move(request.width):
            goal_handle.succeed()
            result.success = True
            logger.info("Gripper Moving succeeded")
        return result


def main(args: str = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    gripper_action_controller_client = GripperActionControllerClient()
    executor.add_node(gripper_action_controller_client)

    fr3_gripper = FrankaGripperSimulation(gripper_action_controller_client)
    executor.add_node(fr3_gripper)

    spin_executor(executor)


if __name__ == "__main__":
    main()
