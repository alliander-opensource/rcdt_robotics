# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from rclpy.node import Node
from rclpy.client import Client

TIME_OUT = 3


@dataclass
class Action:
    service_name: str
    service_type: object
    client: Client | None = None

    def call(self, node: Node) -> bool:
        if self.client is None:
            self.client = node.create_client(self.service_type, self.service_name)

        if not self.client.wait_for_service(TIME_OUT):
            node.get_logger().error(f"Service `{self.service_name}` is not available.")
            return False

        request = self.service_type.Request()
        response = self.client.call(request)
        return response.success


@dataclass
class Sequence:
    name: str
    actions: list[Action]
    success = True
    index = 0

    def reset(self) -> None:
        self.success = True
        self.index = 0

    def is_finished(self) -> bool:
        return not self.success or self.index >= len(self.actions)

    def current_action(self) -> Action:
        return self.actions[self.index]

    def log_start(self, node: Node) -> None:
        node.get_logger().info(f"Starting execution of sequence '{self.name}'.")

    def log_progress(self, node: Node) -> None:
        if self.success:
            node.get_logger().info(f"Finished: {self.current_action().service_name}")
        else:
            node.get_logger().error(f"Failed: {self.current_action().service_name}")

    def log_result(self, node: Node) -> None:
        if self.success:
            node.get_logger().info(f"Sequency '{self.name}' was executed successfully.")
        else:
            node.get_logger().error(f"Failed to execute sequence '{self.name}'.")

    def call(self, node: Node) -> None:
        action = self.actions[self.index]
        self.success = action.call(node)

    def execute(self, node: Node) -> bool:
        self.reset()
        self.log_start(node)
        while not self.is_finished():
            self.call(node)
            self.log_progress(node)
            self.index += 1
        self.log_result(node)
        return self.success
