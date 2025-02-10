# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from rclpy.node import Node
from rclpy.client import Client
from rclpy.action.server import ServerGoalHandle
from rcdt_messages.action import Sequence as SequenceMsg


TIME_OUT = 3


def info(node: Node, message: str) -> None:
    node.get_logger().info(message)


def warn(node: Node, message: str) -> None:
    node.get_logger().warn(message)


def error(node: Node, message: str) -> None:
    node.get_logger().error(message)


@dataclass
class Action:
    service_name: str
    service_type: object
    args: dict | None = None
    links: dict | None = None
    node: Node | None = None
    client: Client | None = None

    def set_args(self, args: dict) -> "Action":
        self.args = args
        return self

    def set_links(self, links: dict) -> "Action":
        self.links = links
        return self

    def fill_from_arguments(self, request: object) -> bool:
        if self.args is None:
            return True
        for key, value in self.args.items():
            if not hasattr(request, key):
                warn(self.node, f"Skipped '{key}': doesn't exist on {type(request)}")
                continue
            try:
                setattr(request, key, value)
            except AssertionError as e:
                warn(self.node, f"Skipped '{key}' because of wrong type:\n {e}")
                return False
        return True

    def fill_from_last_result(self, request: object, last_result: object) -> bool:
        if self.links is None:
            return True
        for arg_out, arg_in in self.links.items():
            if not (hasattr(last_result, arg_out) and hasattr(request, arg_in)):
                warn(self.node, f"Link {arg_out}->{arg_in} is not possible.")
                return False
            try:
                value = getattr(last_result, arg_out)
            except Exception as e:
                error(str(e))
                return False
            try:
                setattr(request, arg_in, value)
            except Exception as e:
                error(str(e))
                return False
            return True

    def call(self, last_result: object) -> tuple[bool, object]:
        if self.client is None:
            self.client = self.node.create_client(self.service_type, self.service_name)

        request = self.service_type.Request()

        if not self.fill_from_arguments(request):
            return False, None
        if not self.fill_from_last_result(request, last_result):
            return False, None
        if not self.client.wait_for_service(TIME_OUT):
            error(self.node, f"Service `{self.service_name}` is not available.")
            return False, None

        response = self.client.call(request)
        return response.success, response


@dataclass
class Sequence:
    name: str
    actions: list[Action]
    node: Node | None = None
    goal_handle: ServerGoalHandle | None = None
    last_result = None
    success = True
    index = 0

    def reset(self) -> None:
        self.success = True
        self.index = 0

    def is_finished(self) -> bool:
        return not self.success or self.index >= len(self.actions)

    def current_action(self) -> Action:
        return self.actions[self.index]

    def log_start(self) -> None:
        info(self.node, f"Starting execution of sequence '{self.name}'.")

    def log_progress(self) -> None:
        feedback = SequenceMsg.Feedback()
        feedback.success = self.success
        if self.success:
            message = f"Finished: {self.current_action().service_name}"
            info(self.node, message)
        else:
            message = f"Failed: {self.current_action().service_name}"
            error(self.node, message)
        feedback.message = message
        self.goal_handle.publish_feedback(feedback)

    def log_result(self) -> None:
        if self.success:
            info(self.node, f"Sequency '{self.name}' was executed successfully.")
        else:
            error(self.node, f"Failed to execute sequence '{self.name}'.")

    def call(self) -> None:
        action = self.actions[self.index]
        action.node = self.node
        self.success, self.last_result = action.call(self.last_result)

    def execute(self) -> bool:
        self.reset()
        self.log_start()
        while not self.is_finished():
            self.call()
            self.log_progress()
            self.index += 1
        self.log_result()
        return self.success
