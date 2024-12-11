#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy import logging
from franka_msgs.action import Move
from std_srvs.srv import Trigger

ros_logger = logging.get_logger(__name__)


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

    def callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = self.open_gripper()
        return response

    def open_gripper(self) -> bool:
        goal = Move.Goal()
        goal.width = 0.08
        goal.speed = 0.03

        if not self.client.wait_for_server(timeout_sec=3):
            self.get_logger().error("Gripper move client not available.")
            return False

        result: Move.Impl.GetResultService.Response = self.client.send_goal(goal)
        if not result.result.success:
            self.get_logger().error("Opening gripper did not succeed.")
        return result.result.success


def main(args: str = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = OpenGripper()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        ros_logger.info("Keyboard interrupt, shutting down.\n")
    except Exception as e:
        raise e
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
