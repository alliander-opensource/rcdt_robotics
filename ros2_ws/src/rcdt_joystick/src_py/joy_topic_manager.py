#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass

import mashumaro.codecs.yaml as yaml_codec
import rclpy
from rcdt_utilities.launch_utils import get_file_path, get_yaml, spin_node
from rclpy.node import Node, Publisher
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# Define the latched QoS profile
latched_qos = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
)


@dataclass
class Output:
    """Data class representing a joystick output configuration.

    Attributes:
        button (int): The index of the joystick button to monitor.
        topic (str | None): The topic to publish the Joy message to when the button state changes.
        moveit (bool): Whether this output is intended for MoveIt! integration.
        state (int): The current state of the button (pressed or not).
    """

    button: int
    topic: str | None = None
    moveit: bool = False
    state: int = 0

    def state_changed(self, state: bool) -> bool:
        """Check if the state of the button has changed.

        Args:
            state (bool): The current state of the button (pressed or not).

        Returns:
            bool: True if the state has changed, False otherwise.
        """
        if self.state == state:
            return False
        self.state = state
        return True


class JoyTopicManager(Node):
    """A ROS2 node that manages joystick topics based on button presses.

    This node subscribes to a joystick input topic and publishes Joy messages to specific topics
    based on the configuration defined in a YAML file. It allows dynamic topic switching
    based on joystick button states.
    """

    def __init__(self) -> None:
        """Initialize the JoyTopicManager node."""
        super().__init__("joy_topic_manager")
        self.declare_parameter("joy_topic", value="/joy")
        joy_input = self.get_parameter("joy_topic").get_parameter_value().string_value

        file = get_file_path("rcdt_joystick", ["config"], "joy_topics.yaml")
        yaml = get_yaml(file)

        self.outputs = [yaml_codec.decode(str(yaml[output]), Output) for output in yaml]
        self.pubs: dict[str, Publisher] = {}
        for output in self.outputs:
            if output.topic is not None:
                self.pubs[output.topic] = self.create_publisher(Joy, output.topic, 10)

        self.topic = None
        self.create_subscription(Joy, joy_input, self.handle_joy_message, 10)
        self.state_pub = self.create_publisher(
            String, "~/state", qos_profile=latched_qos
        )

    def handle_joy_message(self, msg: Joy) -> None:
        """Handle incoming Joy messages and apply output changes based on button states.

        Args:
            msg (Joy): The incoming Joy message containing button states.
        """
        self.apply_output_changes(msg)
        self.pass_joy_message(msg)

    def apply_output_changes(self, msg: Joy) -> None:
        """Apply changes to the output topics based on the current Joy message.

        Args:
            msg (Joy): The incoming Joy message containing button states.
        """
        for output in self.outputs:
            if output.button >= len(msg.buttons):
                self.get_logger().warn(
                    f"Button index {output.button} out of range in Joy message with {len(msg.buttons)} buttons."
                )
                continue
            state = msg.buttons[output.button]
            if output.state_changed(state):
                self.topic_changed(output.topic)

    def topic_changed(self, topic: str) -> bool:
        """Change the currently active topic and publish the new state.

        Args:
            topic (str): The new topic to which Joy messages should be published.

        Returns:
            bool: True if the topic was changed, False if it remains the same.
        """
        if self.topic == topic:
            return False
        self.topic = topic

        msg = String()
        msg.data = self.topic if self.topic is not None else ""
        self.state_pub.publish(msg)

        if self.topic is None:
            self.get_logger().info("Joy topic passing stopped.")
        else:
            self.get_logger().info(f"Joy topic is now passed to {self.topic}.")
        return True

    def pass_joy_message(self, msg: Joy) -> None:
        """Pass the Joy message to the currently active topic.

        Args:
            msg (Joy): The Joy message to be published.
        """
        if self.topic is None:
            return
        pub = self.pubs[self.topic]
        pub.publish(msg)


def main(args: list | None = None) -> None:
    """Main function to initialize the JoyTopicManager node and start spinning it.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    node = JoyTopicManager()
    spin_node(node)


if __name__ == "__main__":
    main()
