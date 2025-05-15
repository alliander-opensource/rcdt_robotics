import subprocess

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events.process import ProcessIO
from launch_ros.actions import Node
from rclpy import logging
from termcolor import cprint

logger = logging.get_logger(__name__)


class Register:
    registered: list["Register"] = []
    on_all_started_action = False

    @staticmethod
    def count_started() -> int:
        """Returns the number of registered nodes that are started."""
        n = 0
        for registered_node in Register.registered:
            if registered_node.is_started:
                n += 1
        return n

    @staticmethod
    def count_registered() -> int:
        """Returns the number of registered nodes."""
        return len(Register.registered)

    @staticmethod
    def progress() -> str:
        """Returns a string '[x/y]' with x the started nodes and y the total registered nodes."""
        return f"[{Register.count_started()}/{len(Register.registered)}]"

    @staticmethod
    def all_started() -> bool:
        """Returns whether all registered nodes are started."""
        return Register.count_started() == Register.count_registered()

    def __init__(self, node: Node):
        self.node = node
        self.is_started = False
        self.message: str
        Register.registered.append(self)

    @classmethod
    def on_log(cls, node: Node, message: str) -> LaunchDescription:
        """Registers the given node to be completely started when it's log contains the given message."""

        registered_node = cls(node)
        registered_node.message = message
        event_handler = RegisterEventHandler(
            OnProcessIO(
                target_action=node,
                on_stderr=registered_node.process_io,
            )
        )
        return LaunchDescription([node, event_handler])

    @classmethod
    def on_exit(cls, node: Node) -> LaunchDescription:
        """Registers the given node to be completely started when it exists."""
        registered_node = cls(node)
        event_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=node, on_exit=lambda *_: registered_node.started()
            )
        )
        return LaunchDescription([node, event_handler])

    @staticmethod
    def on_all_started(node: Node) -> LaunchDescription:
        """
        Starts a wait_for_topic node that waits for an Empty message on /start topic.
        This message is published when all registered nodes are started, resulting in the wait_for_topic node to exit.
        This exit triggers the start of the node passed to this function.
        """
        Register.on_all_started_action = True

        wait_for_topic = Node(
            package="rcdt_utilities",
            executable="wait_for_topic.py",
            parameters=[{"topic": "/start"}, {"msg_type": "Empty"}, {"log": False}],
        )

        event_handler = RegisterEventHandler(
            OnProcessExit(target_action=wait_for_topic, on_exit=node)
        )
        return LaunchDescription([wait_for_topic, event_handler])

    def process_io(self, event: ProcessIO) -> None:
        if self.is_started:
            return
        if self.message in event.text.decode():
            self.started()

    def started(self) -> None:
        self.is_started = True
        cprint(
            f"[Registered Nodes] {Register.progress()}: {self.node.name} started successfully!",
            "blue",
        )
        if Register.all_started():
            self.all_started_action()

    def all_started_action(self) -> None:
        cprint("[Registered Nodes] : All nodes started!", "blue")
        if Register.on_all_started_action:
            subprocess.run(
                ["ros2", "topic", "pub", "--once", "/start", "std_msgs/msg/Empty"],
                check=False,
                stdout=subprocess.DEVNULL,
            )
