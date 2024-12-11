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
from franka_msgs.action import Grasp
from std_srvs.srv import Trigger

ros_logger = logging.get_logger(__name__)


class OpenGripper(Node):
    def __init__(self) -> None:
        super().__init__("close_gripper")

        cbg_service = MutuallyExclusiveCallbackGroup()
        self.create_service(
            Trigger, "close_gripper", self.callback, callback_group=cbg_service
        )

        cbg_client = MutuallyExclusiveCallbackGroup()
        self.client = ActionClient(
            self, Grasp, "/fr3_gripper/grasp", callback_group=cbg_client
        )

    def callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = self.close_gripper()
        return response

    def close_gripper(self) -> bool:
        goal = Grasp.Goal()
        goal.width = 0.0
        goal.epsilon.inner = goal.epsilon.outer = 0.08
        goal.force = 100.0
        goal.speed = 0.03

        if not self.client.wait_for_server(timeout_sec=2):
            self.get_logger().error("Gripper grasp client not available.")
            return False

        result: Grasp.Impl.GetResultService.Response = self.client.send_goal(goal)
        if not result.result.success:
            self.get_logger().error("Closing gripper did not succeed.")
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
