#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

MAX_TORQUES = [100.0, 100.0, 100.0, 80.0, 80.0, 40.0, 40.0]
MAX_FORCES = [100.0, 100.0, 100.0, 30.0, 30.0, 30.0]


class StartArm(Node):
    """Node to start the UR robot arm."""

    def __init__(self):
        """Initialize the StartArm node."""
        super().__init__("ur_start_arm")
        namespace = self.get_namespace().lstrip("/")
        self.power_on_client = self.create_client(
            Trigger, f"/{namespace}/dashboard_client/power_on"
        )
        self.brake_release_client = self.create_client(
            Trigger, f"/{namespace}/dashboard_client/brake_release"
        )
        self.success = True

        self.power_on()
        self.brake_release()
        if self.success:
            self.get_logger().info("UR robot arm started successfully.")

    def power_on(self) -> None:
        """Power on the UR robot arm."""
        if not self.success:
            return
        while not self.power_on_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        request = Trigger.Request()
        future = self.power_on_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: Trigger.Response = future.result()
        self.success = response.success

        if response.success:
            self.get_logger().info("UR robot arm powered on successfully.")
        else:
            self.get_logger().warn("Failed to power on UR robot arm.")

    def brake_release(self) -> None:
        """Release the brakes of the UR robot arm."""
        if not self.success:
            return
        while not self.brake_release_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        request = Trigger.Request()
        future = self.brake_release_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response: Trigger.Response = future.result()
        self.success = response.success

        if response.success:
            self.get_logger().info("UR robot arm brakes released successfully.")
        else:
            self.get_logger().warn("Failed to release UR robot arm brakes.")


def main(args: list | None = None) -> None:
    """Main function to start the UR robot arm.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    start_arm = StartArm()
    start_arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
