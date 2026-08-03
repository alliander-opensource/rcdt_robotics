#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# Based on: https://github.com/baha2r/robotiq3f_py/blob/main/robotiqcontrol/GripperController.py


import time

import rclpy
from alliander_interfaces.action import StringAction
from alliander_robotiq.modbus_controller import (
    ModbusController,
    ModbusControllerSimulation,
    finger_joint_position_from_bit,
)
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

RATE = 0.1
TIMEOUT = 15


class RobotiqController(Node):
    """Node to control the Robotiq 3-finger gripper."""

    def __init__(self) -> None:
        """Initialize node."""
        super().__init__("gripper_controller")
        self.declare_parameter("simulation", False)
        self.declare_parameter("ip", "")
        self.declare_parameter("port", -1)
        simulation = self.get_parameter("simulation").get_parameter_value().bool_value
        ip = self.get_parameter("ip").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value

        if not simulation and (not ip or port == -1):
            self.get_logger().error(
                "IP address and port must be set in the parameters when using hardware."
            )
            return

        self.status_publisher = self.create_publisher(String, "status", 10)
        self.joint_state_publisher = self.create_publisher(
            JointState, "joint_states", 10
        )
        simulation_controller_publisher = self.create_publisher(
            Float64MultiArray, "position_controller/commands", 10
        )

        if simulation:
            self.modbus_controller = ModbusControllerSimulation(
                simulation_controller_publisher
            )
        else:
            self.modbus_controller = ModbusController(ip, port)

            self.get_logger().info("Connecting with gripper...")
            connected = self.modbus_controller.open()
            if connected:
                self.get_logger().info("Connection successful.")
            else:
                self.get_logger().error("Connection failed.")

            self.timer = self.create_timer(RATE, self.update_callback)

        self.modbus_controller.send_command()
        self.action_server = ActionServer(
            self, StringAction, "~/action", self.action_callback
        )
        self.get_logger().info("RobotIQ controller initialized.")

    def update_callback(self) -> None:
        """Update the status of the gripper."""
        self.modbus_controller.read_status()
        msg = String()
        msg.data = str(self.modbus_controller.status.to_json())
        self.status_publisher.publish(msg)
        self.update_joint_state()

    def update_joint_state(self) -> None:
        """Update the joint state of the gripper."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "finger_1_joint_1",
            "finger_1_joint_2",
            "finger_1_joint_3",
            "finger_2_joint_1",
            "finger_2_joint_2",
            "finger_2_joint_3",
            "finger_middle_joint_1",
            "finger_middle_joint_2",
            "finger_middle_joint_3",
            "palm_finger_1_joint",
            "palm_finger_2_joint",
        ]

        joint_positions_request = []
        for finger in self.modbus_controller.status.fingers:
            for joint in range(finger.number_of_joints):
                position = finger.position if finger.position else 0
                joint_positions_request.append(
                    finger_joint_position_from_bit(position, joint + 1, finger.scissor)
                )

        msg.position = joint_positions_request
        self.joint_state_publisher.publish(msg)

    def action_callback(self, goal_handle: ServerGoalHandle) -> StringAction.Result:  # ruff: ignore[too-many-branches]
        """Trigger the requested gripper action.

        Args:
            goal_handle (ServerGoalHandle): The goal handle for the action server.

        Returns:
            StringAction.Result: The result of the action server execution.
        """
        request: StringAction.Goal = goal_handle.request
        result = StringAction.Result()

        match request.text:
            case "activate":
                self.modbus_controller.send_command()
            case "reset":
                self.modbus_controller.send_command(activate=False)
            case "open":
                self.modbus_controller.send_command(position=0)
            case "close":
                self.modbus_controller.send_command(position=255)
            case "pinch-open":
                self.modbus_controller.send_command(mode="pinch", position=0)
            case "pinch-close":
                self.modbus_controller.send_command(mode="pinch", position=255)
            case "wide-open":
                self.modbus_controller.send_command(mode="wide", position=0)
            case "wide-close":
                self.modbus_controller.send_command(mode="wide", position=255)
            case _:
                self.get_logger().error(f"Unknown command: {request.text}")
                result.success = False
                result.message = f"Unknown command: {request.text}"
                goal_handle.abort()
                return result

        time.sleep(1)
        start_time = time.time()
        while self.modbus_controller.status.motion_status == "in_motion":
            time.sleep(0.1)
            if time.time() - start_time > TIMEOUT:
                result.success = False
                result.message = "Timeout reached while waiting for action to complete."
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

        if self.modbus_controller.status.motion_status == "reached_target_position":
            result.success = True
            goal_handle.succeed()
        else:
            result.success = False
            result.message = "Failed to reach target position."
            goal_handle.abort()

        return result


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and set the thresholds.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    griper_controller = RobotiqController()
    executor = MultiThreadedExecutor()
    executor.add_node(griper_controller)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        griper_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
