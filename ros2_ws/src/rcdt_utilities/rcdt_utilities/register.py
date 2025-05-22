# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import Union

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.events.process import ProcessIO
from launch_ros.actions import Node
from rclpy import logging
from termcolor import colored

LOGGER = logging.get_logger("Register")
CONF_NAME = "registered_group"


class RegisteredLaunchDescription(IncludeLaunchDescription):
    """Extends on the default IncludeLaunchDescription, to automatically register an unique group_id."""

    def __init__(self, launch_description_source: str, launch_arguments: dict = None):
        if launch_arguments is None:
            launch_arguments = {}
        self.group_id = Register.get_unique_group_id()
        launch_arguments[CONF_NAME] = self.group_id
        super().__init__(
            launch_description_source,
            launch_arguments=launch_arguments.items(),
        )


class Register:
    """A class to register the individual processes from launch files, to ensure a desired launch order."""

    group_id = 0
    register: list[Union["Register", str]] = []
    actions = 0
    started = 0
    all_started = False

    @staticmethod
    def get_unique_group_id() -> str:
        """Get a unique group id based on a ascending counter."""
        Register.group_id += 1
        return f"group_{Register.group_id}"

    @staticmethod
    def reset() -> None:
        """Reset the register. Useful for pytest since it can launch ros multiple times in the same session."""
        Register.group_id = 0
        Register.register = []
        Register.actions = 0
        Register.started = 0

    @staticmethod
    def next(*_: any) -> Node | ExecuteProcess:
        """
        Returns the executable of the next registered item that should start.

        This item is located at position 1 of the register list, since position 0 contains the initial register that starts the chain.
        An item can also be a string representing a group. Therefore we pop until the item at position 1 is of type Register.
        There are no registered items left to start when the length of the register list equals 1, so then we return en empty launch description.
        """
        item = None
        Register.started += 1
        while not isinstance(item, Register):
            if len(Register.register) == 1:
                log_progress()
                Register.all_started = True
                Register.reset()
                return LaunchDescription([])
            item = Register.register.pop(1)
        log_progress(item.action)
        return item.action

    @staticmethod
    def group(
        launch_description: RegisteredLaunchDescription, context: LaunchContext
    ) -> RegisteredLaunchDescription:
        """
        Adds a the group id of the included launch_description to the register.

        This enables the items of the included launch description to register in the right order later.
        """
        name = launch_description.group_id
        group = context.launch_configurations.get(CONF_NAME)

        if group:
            index = Register.register.index(group)
            Register.register.insert(index, name)
        else:
            Register.register.append(name)
        return launch_description

    @staticmethod
    def connect_context(
        groups: list[RegisteredLaunchDescription],
    ) -> LaunchDescription:
        """
        Returns a LaunchDescription of an OpaqueFunction, so that LaunchContext is available.

        This LaunchContext is required for the Register methods to correctly link the different groups of included launch files.
        """

        def launch_setup(
            context: LaunchContext,
            registered_launch_descriptions: list[RegisteredLaunchDescription],
        ) -> LaunchDescription:
            launch_items = []
            for description in registered_launch_descriptions:
                if not isinstance(description, RegisteredLaunchDescription):
                    raise TypeError
                launch_items.append(Register.group(description, context))
            return launch_items

        opaque_function = OpaqueFunction(function=launch_setup, args=[groups])
        return LaunchDescription([opaque_function])

    def __init__(self):
        self.action: Union[Node | ExecuteProcess]

        self.is_started = False
        self.log: str

    @classmethod
    def on_start(
        cls, action: Union[Node, ExecuteProcess], context: LaunchContext
    ) -> LaunchDescription:
        """Registers the given action to be ready directly on start."""
        register = cls()
        event_handler = RegisterEventHandler(
            OnProcessStart(target_action=action, on_start=Register.next)
        )
        return register.insert_action(action, event_handler, context)

    @classmethod
    def on_exit(
        cls, action: Union[Node, ExecuteProcess], context: LaunchContext
    ) -> LaunchDescription:
        """Registers the given action to be ready on exit."""
        register = cls()
        event_handler = RegisterEventHandler(
            OnProcessExit(target_action=action, on_exit=Register.next)
        )
        return register.insert_action(action, event_handler, context)

    @classmethod
    def on_log(
        cls, action: Union[Node, ExecuteProcess], log: str, context: LaunchContext
    ) -> LaunchDescription:
        """Registers the given action to be ready when logging the given log message."""
        register = cls()
        register.log = log
        event_handler = RegisterEventHandler(
            OnProcessIO(target_action=action, on_stderr=register.process_io)
        )

        return register.insert_action(action, event_handler, context)

    def process_io(self, event: ProcessIO) -> None:
        """Returns the next register to start if the defined log is captured."""
        if self.is_started:
            return
        if self.log in event.text.decode():
            self.started = True
            return Register.next()

    def insert_action(
        self,
        action: Union[Node, ExecuteProcess],
        event_handler: RegisterEventHandler,
        context: LaunchContext,
    ) -> LaunchDescription:
        """
        Inserts the action in the register and returns a launch description.

        If a group is defined, the index is defined based on the index of the group.
        Otherwise, the index is simply the length of the register, so that the item is appended at the end.

        A launch description is returned with the event_handler.
        If the item is the first action to start, the launch_description also contains the action to trigger the chain reaction.
        """
        self.action = action
        group = context.launch_configurations.get(CONF_NAME)
        index = Register.register.index(group) if group else len(Register.register)
        Register.register.insert(index, self)
        Register.actions += 1

        if index == 0:
            log_progress(action)
            Register.all_started = False
            return LaunchDescription([action, event_handler])
        else:
            return LaunchDescription([event_handler])


def log_progress(action: Union[Node, ExecuteProcess] = None) -> None:
    """Log the start (INIT), progress ([started]/[registerd]) and when finished (ALL READY)."""
    if Register.started == 0:
        msg = "[START] "
    else:
        msg = f"[{Register.started}/{Register.actions}] "

    if isinstance(action, Node):
        msg += "(node) " + action.node_package + " " + action.node_executable
    elif isinstance(action, ExecuteProcess):
        msg += "(process) "
        for part in action.cmd:
            msg += part[0].text + " "
    elif not action:
        msg += "All actions are started!"

    LOGGER.info(colored(msg, "blue"))
