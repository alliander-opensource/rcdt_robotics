# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass

from rcdt_messages.action import Sequence as SequenceMsg
from rclpy.action.server import ServerGoalHandle
from rclpy.client import Client
from rclpy.node import Node

TIME_OUT = 3


def info(node: Node, message: str) -> None:
    """Log an info message using the provided node's logger.

    Args:
        node (Node): The ROS 2 node to use for logging.
        message (str): The message to log.

    """
    node.get_logger().info(message)


def warn(node: Node, message: str) -> None:
    """Warn about a message using the provided node's logger.

    Args:
        node (Node): The ROS 2 node to use for logging.
        message (str): The message to log.

    """
    node.get_logger().warn(message)


def error(node: Node, message: str) -> None:
    """Log an error message using the provided node's logger.

    Args:
        node (Node): The ROS 2 node to use for logging.
        message (str): The message to log.

    """
    node.get_logger().error(message)


@dataclass
class Action:
    """Class representing an action to be executed in a sequence.

    Attributes:
        service_name (str): The name of the service to call.
        service_type (object): The type of the service to call.
        args (dict | None): A dictionary of arguments to pass to the service.
        links (dict | None): A dictionary of links where keys are output arguments and values are input arguments.
        node (Node | None): The ROS 2 node associated with the action.
        client (Client | None): The ROS 2 client for the service.

    """

    service_name: str
    service_type: object
    args: dict | None = None
    links: dict | None = None
    node: Node | None = None
    client: Client | None = None

    def set_args(self, args: dict) -> "Action":
        """Set the arguments for the action.

        Args:
            args (dict): A dictionary of arguments to set for the action.

        Returns:
            Action: The current instance of the Action class with updated arguments.

        """
        self.args = args
        return self

    def set_links(self, links: dict) -> "Action":
        """Set the links for the action.

        Args:
            links (dict): A dictionary of links where keys are output arguments and values are input arguments.

        Returns:
            Action: The current instance of the Action class with updated links.

        """
        self.links = links
        return self

    def fill_from_arguments(self, request: object) -> bool:
        """Fill the request object with arguments from the action.

        Args:
            request (object): The request object to fill with arguments.

        Returns:
            bool: True if the request was filled successfully, False otherwise.

        """
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
        """Fill the request object with values from the last result based on links.

        Args:
            request (object): The request object to fill with values from the last result.
            last_result (object): The last result object containing values to fill into the request.

        Returns:
            bool: True if the request was filled successfully, False otherwise.

        """
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
        """Call the service associated with the action.

        Args:
            last_result (object): The last result object to use for filling the request.

        Returns:
            tuple[bool, object]: A tuple containing a boolean indicating success and the response object.

        """
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
    """Class representing a sequence of actions to be executed.

    Attributes:
        name (str): The name of the sequence.
        actions (list[Action]): A list of actions to be executed in the sequence.
        node (Node | None): The ROS 2 node associated with the sequence.
        goal_handle (ServerGoalHandle | None): The goal handle for the action server.
        last_result (object): The last result from the executed action.
        success (bool): Indicates whether the sequence was executed successfully.
        index (int): The current index of the action being executed in the sequence.

    """

    name: str
    actions: list[Action]
    node: Node | None = None
    goal_handle: ServerGoalHandle | None = None
    last_result = None
    success = True
    index = 0

    def reset(self) -> None:
        """Reset the sequence to its initial state."""
        self.success = True
        self.index = 0

    def is_finished(self) -> bool:
        """Check if the sequence has finished executing.

        Returns:
            bool: True if the sequence is finished, False otherwise.

        """
        return not self.success or self.index >= len(self.actions)

    def current_action(self) -> Action:
        """Get the current action being executed in the sequence.

        Returns:
            Action: The current action object.

        """
        return self.actions[self.index]

    def log_start(self) -> None:
        """Log the start of the sequence execution."""
        info(self.node, f"Starting execution of sequence '{self.name}'.")

    def log_progress(self) -> None:
        """Log the progress of the sequence execution.Publishes feedback to the goal handle if available. If the sequence is successful, logs a success message; otherwise, logs a failure message."""
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
        """Log the result of the sequence execution. If the sequence was successful, logs a success message; otherwise, logs a failure message."""
        if self.success:
            info(self.node, f"Sequency '{self.name}' was executed successfully.")
        else:
            error(self.node, f"Failed to execute sequence '{self.name}'.")

    def call(self) -> None:
        """Call the current action in the sequence. Sets the node for the action and calls the action's `call` method. Updates the `success` and `last_result` attributes based on the action's result."""
        action = self.actions[self.index]
        action.node = self.node
        self.success, self.last_result = action.call(self.last_result)

    def execute(self) -> bool:
        """Execute the sequence of actions. Resets the sequence, logs the start, and iterates through the actions until finished. Logs the progress after each action call and logs the final result.

        Returns:
            bool: True if the sequence was executed successfully, False otherwise.

        """
        self.reset()
        self.log_start()
        while not self.is_finished():
            self.call()
            self.log_progress()
            self.index += 1
        self.log_result()
        return self.success
