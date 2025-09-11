#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import math
import random
import subprocess

import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from rclpy.node import Node

# --- simple knobs ---
AREA_MIN_X, AREA_MAX_X = -2.0, 2.0
AREA_MIN_Y, AREA_MAX_Y = -2.0, 2.0
KP = 0.8  # proportional gain
V_MAX = 0.5  # clamp speed
TARGET_RADIUS = 0.2  # reached when closer than this (m)
STOP_DISTANCE = 1.5  # stop if panther closer than this (m)


class MovingBlockPOC(Node):
    """Node for controlling a moving block in a Gazebo simulation."""

    def __init__(self):
        """Node for controlling a moving block in a Gazebo simulation."""
        super().__init__("moving_block_poc")
        self.bridge_proc = subprocess.Popen(
            [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                "/model/moving_block/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/model/moving_block/pose@geometry_msgs/msg/Pose@gz.msgs.Pose",
                "/model/panther/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        self.get_logger().info("Bridging: cmd_vel + poses")
        # ROS I/O (assumes topics are already bridged)
        self.pub_cmd = self.create_publisher(Twist, "/model/moving_block/cmd_vel", 10)
        self.sub_block = self.create_subscription(
            Pose, "/model/moving_block/pose", self._on_block_pose, 10
        )
        self.sub_panther = self.create_subscription(
            PoseArray, "/model/panther/pose", self._on_panther_pose, 10
        )

        # State
        self.block_xy = None
        self.panther_xy = None
        self.target = None
        self._new_target()  # set first target
        # 10 Hz control
        self.create_timer(0.05, self._loop)

    def _on_block_pose(self, msg: Pose) -> None:
        """Callback for the moving block's pose.

        Args:
            msg (Pose): The pose message from the moving block.
        """
        # ignore occasional zero pose
        if msg.position.x == 0.0 and msg.position.y == 0.0 and msg.position.z == 0.0:
            return
        self.block_xy = (msg.position.x, msg.position.y)

    def _on_panther_pose(self, msg: PoseArray) -> None:
        """Callback for the panther's pose.

        Args:
            msg (PoseArray): The pose array message from the panther.
        """
        self.panther_xy = (msg.poses[0].position.x, msg.poses[0].position.y)

    def _new_target(self) -> None:
        """Set a new random target position within the defined area."""
        self.target = (
            random.uniform(AREA_MIN_X, AREA_MAX_X),
            random.uniform(AREA_MIN_Y, AREA_MAX_Y),
        )
        self.get_logger().info(f"New target: {self.target}")

    def _loop(self) -> None:
        """Control loop for the moving block."""
        if self.block_xy is None:
            return

        twist = Twist()  # zero by default

        # Stop if panther is too close
        if self.panther_xy is not None:
            bx, by = self.block_xy
            px, py = self.panther_xy
            self.get_logger().info(
                f"Panther is {math.hypot(px - bx, py - by):.2f} m away"
            )
            if math.hypot(px - bx, py - by) < STOP_DISTANCE:
                self.pub_cmd.publish(twist)
                return

        # Move toward target (world-frame P controller)
        tx, ty = self.target
        bx, by = self.block_xy
        dx, dy = tx - bx, ty - by
        dist = math.hypot(dx, dy)

        if dist < TARGET_RADIUS:
            self.get_logger().info(f"Reached {self.target} (dist {dist:.2f} m)")
            self._new_target()
            self.pub_cmd.publish(twist)  # brief stop
            return

        # P control + clamp
        vx, vy = KP * dx, KP * dy
        speed = math.hypot(vx, vy)
        if speed > V_MAX:
            s = V_MAX / max(speed, 1e-9)
            vx *= s
            vy *= s

        self.get_logger().info(
            f"Moving to {self.target} (current position: {self.block_xy}, target distance: {dist:.2f})"
        )
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = 0.0
        self.pub_cmd.publish(twist)


def main() -> None:
    """Main entry point for the moving block node."""
    rclpy.init()
    node = MovingBlockPOC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
