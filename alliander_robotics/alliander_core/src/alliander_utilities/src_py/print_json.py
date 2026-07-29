#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import curses
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PrintJSON(Node):
    """A ROS 2 node that subscribes to a topic and prints incoming JSON data in a curses window."""

    def __init__(self):
        """Initialize."""
        super().__init__("json_printer")
        self.declare_parameter("topic", "")
        topic = self.get_parameter("topic").get_parameter_value().string_value

        if not topic:
            self.get_logger().error(
                "No topic provided. Please set the 'topic' parameter."
            )
            return

        self.data = {}
        self.create_subscription(String, topic, self.callback, 10)
        threading.Thread(target=rclpy.spin, args=(self,)).start()
        curses.wrapper(self.update_window)

    def callback(self, msg: String) -> None:
        """Callback function to update the data.

        Args:
            msg (String): The incoming message containing JSON data.
        """
        self.data = json.loads(msg.data)

    def update_window(self, window: curses.window) -> None:
        """Update the curses window with the latest JSON data.

        Args:
            window (curses.window): The curses window to update.
        """
        window.nodelay(True)
        window.timeout(100)

        counter: int = 0

        while True:
            window.clear()

            # Render the JSON data
            row: int = 0
            for key, value in self.data.items():
                window.addstr(row, 0, f"{key}: {value}")
                row += 1

            window.addstr(row + 1, 0, "Press 'q' to quit.")
            window.refresh()

            # Increment our data counter
            counter += 1

            # Check for user input:
            try:
                key: str = window.getkey()
                if key == "q":
                    break
            except curses.error:
                continue


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    json_printer = PrintJSON()
    json_printer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
