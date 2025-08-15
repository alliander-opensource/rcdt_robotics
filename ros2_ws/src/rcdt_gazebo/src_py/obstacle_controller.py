#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import select
import signal
import subprocess
import sys
import termios
import time
import tty
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile

TOPIC = "/model/moving_block/cmd_vel"
LIN = 0.80  # m/s (forward/back)
ANG = 0.80  # rad/s (yaw)
RATE = 60.0  # Hz publish rate

HELP = """\
Controls:  w = forward   s = back   a = rotate left   d = rotate right
           SPACE = stop  q = quit
"""


def kbhit(timeout: float = 0.0) -> bool:
    """Check if a keypress is available within the given timeout.

    Args:
        timeout (float): Time to wait in seconds.

    Returns:
        bool: True if a key is pressed, False otherwise.
    """
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return bool(r)


class SimpleTeleop(Node):
    """Simple teleoperation node for controlling a Gazebo model."""

    def __init__(self) -> None:
        """Initialize the teleop node and publisher."""
        super().__init__("simple_teleop")
        qos = QoSProfile(depth=1)  # drop old commands
        self.pub = self.create_publisher(Twist, TOPIC, qos)
        self.vx, self.wz = 0.0, 0.0
        self.timer = self.create_timer(1.0 / RATE, self._tick)

    def _tick(self) -> None:
        """Publish the current velocity command."""
        msg = Twist()
        msg.linear.x = self.vx
        msg.angular.z = self.wz
        self.pub.publish(msg)

    def set_cmd(self, vx: float, wz: float) -> None:
        """Set the velocity command.

        Args:
            vx (float): Linear velocity in m/s.
            wz (float): Angular velocity in rad/s.
        """
        self.vx, self.wz = vx, wz

    def stop_now(self) -> None:
        """Immediately stop the robot."""
        self.vx, self.wz = 0.0, 0.0
        self._tick()  # publish one last zero immediately


def main() -> None:
    """Run the teleop node and bridge the Gazebo cmd_vel topic."""
    rclpy.init()

    # Start ROS-Gazebo bridge as a subprocess
    bridge_cmd = [
        "ros2",
        "run",
        "ros_gz_bridge",
        "parameter_bridge",
        f"{TOPIC}@geometry_msgs/msg/Twist@gz.msgs.Twist",
    ]
    bridge_proc = subprocess.Popen(bridge_cmd)

    node = SimpleTeleop()

    # Terminal raw mode
    old_tty = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print(HELP, flush=True)

    def cleanup(*_: Any) -> None:
        """Stop the teleop node and clean up resources.

        Args:
            *_ (Any): Unused arguments from the signal handler (signal number and frame).
        """
        try:
            node.stop_now()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tty)
            node.destroy_node()
            rclpy.shutdown()
            bridge_proc.terminate()
            bridge_proc.wait()
            sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        next_tick = time.monotonic()
        period = 1.0 / RATE
        while rclpy.ok():
            if kbhit(0.0):
                k = sys.stdin.read(1)
                if k in {"q", "Q"}:
                    cleanup()
                elif k == "w":
                    node.set_cmd(LIN, 0.0)
                elif k == "s":
                    node.set_cmd(-LIN, 0.0)
                elif k == "a":
                    node.set_cmd(0.0, ANG)
                elif k == "d":
                    node.set_cmd(0.0, -ANG)
                elif k == " ":
                    node.set_cmd(0.0, 0.0)

            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            next_tick += period
            time.sleep(max(0.0, next_tick - now))
    except Exception:
        cleanup()


if __name__ == "__main__":
    main()
