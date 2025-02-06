#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from logging import getLogger

import rclpy
from franka_msgs.action import Move
from rcdt_utilities.launch_utils import spin_executor
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

logger = getLogger(__name__)


class OpenGripper(Node):
    def __init__(self) -> None:
        super().__init__("open_gripper")

        cbg_service = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger, "open_gripper", self.callback, callback_group=cbg_service
        )

        cbg_client = MutuallyExclusiveCallbackGroup()
        self.client = ActionClient(
            self, Move, "/fr3_gripper/move", callback_group=cbg_client
        )

        self.create_goal()

    def create_goal(self) -> None:
        self.goal = Move.Goal()
        self.goal.width = 0.08
        self.goal.speed = 0.03

    def callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = self.open_gripper()
        return response

    def open_gripper(self) -> bool:
        if not self.client.wait_for_server(timeout_sec=3):
            logger.error("Gripper move client not available.")
            return False

        result: Move.Impl.GetResultService.Response = self.client.send_goal(self.goal)
        if not result.result.success:
            logger.error("Opening gripper did not succeed.")
        return result.result.success


def main(args: str = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = OpenGripper()
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
