#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from franka_msgs.srv import SetForceTorqueCollisionBehavior

MAX_TORQUES = [100.0, 100.0, 100.0, 80.0, 80.0, 40.0, 40.0]
MAX_FORCES = [100.0, 100.0, 100.0, 30.0, 30.0, 30.0]


class SettingsSetter(Node):
    def __init__(self):
        super().__init__("settings_setter")
        self.client = self.create_client(
            SetForceTorqueCollisionBehavior,
            "/service_server/set_force_torque_collision_behavior",
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.request = SetForceTorqueCollisionBehavior.Request()

    def set_thresholds(self) -> SetForceTorqueCollisionBehavior.Response:
        self.request.upper_torque_thresholds_nominal = MAX_TORQUES
        self.request.upper_force_thresholds_nominal = MAX_FORCES
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args: str = None) -> None:
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
