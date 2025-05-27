#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from franka_msgs.srv import SetForceTorqueCollisionBehavior
from rclpy.node import Node

MAX_TORQUES = [100.0, 100.0, 100.0, 80.0, 80.0, 40.0, 40.0]
MAX_FORCES = [100.0, 100.0, 100.0, 30.0, 30.0, 30.0]


class SettingsSetter(Node):
    """Node to set the force and torque collision behavior for the Franka robot."""

    def __init__(self):
        """Initialize the SettingsSetter node."""
        super().__init__("settings_setter")
        self.client = self.create_client(
            SetForceTorqueCollisionBehavior,
            "/franka/service_server/set_force_torque_collision_behavior",
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.request = SetForceTorqueCollisionBehavior.Request()

    def set_thresholds(self) -> SetForceTorqueCollisionBehavior.Response:
        """Set the force and torque thresholds for the Franka robot.

        This method sends a request to the service to set the upper torque and force thresholds
        to predefined maximum values.

        Returns:
            SetForceTorqueCollisionBehavior.Response: The response from the service indicating success or failure.
        """
        self.request.upper_torque_thresholds_nominal = MAX_TORQUES
        self.request.upper_force_thresholds_nominal = MAX_FORCES
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and set the thresholds.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    settings_setter = SettingsSetter()
    response = settings_setter.set_thresholds()
    if response.success:
        settings_setter.get_logger().info("Thresholds set successfully.")
    else:
        settings_setter.get_logger().warn("Setting thresholds failed.")
    settings_setter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
