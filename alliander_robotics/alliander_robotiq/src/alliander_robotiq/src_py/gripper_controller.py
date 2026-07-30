#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# Based on: https://github.com/baha2r/robotiq3f_py/blob/main/robotiqcontrol/GripperController.py


import time
import typing
from dataclasses import dataclass, field
from typing import Literal, Optional

import rclpy
from alliander_interfaces.action import StringAction
from mashumaro.mixins.json import DataClassJSONMixin
from pyModbusTCP.client import ModbusClient
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

IP = "192.168.1.11"
PORT = 502
RATE = 0.1
TIMEOUT = 15

MODES = Literal[
    "basic",
    "pinch",
    "wide",
    "scissor",
]

STATUS_GRIPPER = Literal[
    "resetting",
    "activation_in_progress",
    "mode_change_in_progress",
    "changes_complete",
]

STATUS_MOTION = Literal[
    "in_motion",
    "not_all_fingers_reached_target_position",
    "no_fingers_reached_target_position",
    "reached_target_position",
]

STATUS_FINGER = Literal[
    "in_motion",
    "contact_while_opening",
    "contact_while_closing",
    "reached_target_position",
]

STATUS_FAULT = Literal[
    "no_fault",
    "activation_must_be_completed",
    "mode_change_must_be_completed",
    "activation_bit_must_be_set",
    "communication_chip_not_ready",
    "interference_on_scissor",
    "automatic_release_in_progress",
    "activation_fault_verify_interference",
    "changing_mode_fault_verify_interference",
    "automatic_release_completed_reset_required",
]


class RobotiqController(Node):
    """Node to control the Robotiq 3-finger gripper."""

    def __init__(self) -> None:
        """Initialize node."""
        super().__init__("gripper_controller")
        self.declare_parameter("simulation", False)
        simulation = self.get_parameter("simulation").get_parameter_value().bool_value

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
            self.modbus_controller = ModbusController(IP, PORT)

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
            for joint in range(finger.number_of_joints + 1):
                position = finger.position if finger.position else 0
                joint_positions_request.append(
                    finger_joint_position_from_bit(position, joint, finger.scissor)
                )

        msg.position = joint_positions_request
        self.joint_state_publisher.publish(msg)

    def action_callback(self, goal_handle: ServerGoalHandle) -> StringAction.Result:  # ruff: ignore[too-many-branches]
        """Execute the action server callback.

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


class ModbusControllerSimulation:
    """Class that simulates the modbus controller."""

    def __init__(self, publisher: Publisher) -> None:
        """Initialize the Modbus simulation.

        Args:
            publisher (Publisher): The publisher to publish the joint positions to the ROS controller.
        """
        self.publisher = publisher
        self.status = GripperStatus()
        self.status.motion_status = "reached_target_position"  # Set target position reached always in simulation.

    def send_command(
        self, activate: bool = True, mode: MODES = "basic", position: int = 0
    ) -> None:
        """A simplified version of the send_command method that talks with the ROS controller instead of TCP server on the gripper.

        Args:
            activate (bool): Activate the gripper.
            mode (MODES): The mode of the gripper.
            position (int): The target position of the gripper (0 = Open, 255 = Closed).u
        """
        if not activate:
            return
        joint_positions_request = []

        # Define finger joints:
        for _ in range(3):
            for joint in range(3):
                joint_positions_request.append(
                    finger_joint_position_from_bit(position, joint + 1)
                )

        # Define scissor joints:
        match mode:
            case "basic":
                scissor_position = 135
            case "pinch":
                scissor_position = 255
            case "wide":
                scissor_position = 0
        for joint in range(2):
            joint_positions_request.append(
                finger_joint_position_from_bit(scissor_position, joint + 1, True)
            )

        command = Float64MultiArray(data=joint_positions_request)
        self.publisher.publish(command)

    def read_status(self) -> None:
        """Not implemented in simulation. The motion status is always set to "reached_target_position"."""
        pass


class ModbusController(ModbusClient):
    """Class that contains the modbus commands to control the Robotiq 3-finger gripper via Modbus TCP.

    Respects the protocol definitions in chapter 4.4 and 4.5 of the manual:
    https://assets.robotiq.com/website-assets/support_documents/document/3-Finger_PDF_20190221.pdf
    """

    def __init__(self, host: str, port: int) -> None:
        """Initialize the Modbus client.

        Args:
            host (str): The IP address of the Modbus server.
            port (int): The port number of the Modbus server.
        """
        super().__init__(host, port, timeout=5, auto_open=False)
        self.status = GripperStatus()

    def send_command(  # ruff:ignore[too-many-arguments]
        self,
        activate: bool = True,
        mode: MODES = "basic",
        go_to: bool = True,
        automatic_release: bool = False,
        individual_control_fingers: bool = False,
        individual_control_scissor: bool = False,
        position: int = 0,
        speed: int = 0,
        force: int = 0,
    ) -> None:
        """Send a command to the gripper.

        Args:
            activate (bool): Activate the gripper. Will perform auto-calibration if set to True and must remain True afterwards. When set to False, the gripper will reset and clear fault status.
            mode (MODES): The mode of the gripper. The gripper opens completely when the mode is changed, to avoid interference between the fingers.
            go_to (bool): Needs to be True to move the fingers. Only activation and mode change can be done with this set to False.
            automatic_release (bool): When set to True, the gripper will slowly open until all motion axes reach their mechanical limits and sends a fault signal. Overrides all other commands.
            individual_control_fingers (bool): When set to True, the fingers can be controlled individually.
            individual_control_scissor (bool): When set to True, the scissor can be controlled individually. The mode is ignored when this is set to True.
            position (int): The target position of the gripper (0 = Open, 255 = Closed).
            speed (int): The speed of the gripper (0 = 22 mm/s, 255 = 110 mm/s).
            force (int): The force of the gripper (0 = 15 N, 255 = 60 N).

        Raises:
            ValueError: If position, speed, or force are not in the range 0-255.
        """
        act = str(int(activate))
        mod = bin(list(typing.get_args(MODES)).index(mode))[2:].zfill(2)
        gto = str(int(go_to))
        atr = str(int(automatic_release))

        icf = str(int(individual_control_fingers))
        ics = str(int(individual_control_scissor))

        if any(value not in range(0, 256) for value in [position, speed, force]):
            raise ValueError("Position, Speed, and Force must be between 0 and 255.")

        registers = [
            self.register(self.action(act, mod, gto, atr), self.options(icf, ics)),
            self.register(self.empty(), self.position(position)),
            self.register(self.speed(speed), self.force(force)),
        ]

        self.write_multiple_registers(0, registers)

    def read_status(self) -> None:
        """Read the status of the gripper."""
        registers = self.read_input_registers(0, 8)
        if not registers:
            return

        # Unpack the registers into their respective bytes:
        gripper_status, object_status = self.bytes(registers[0])
        fault_status, position_a_request = self.bytes(registers[1])
        position_a, current_a = self.bytes(registers[2])
        position_b_request, position_b = self.bytes(registers[3])
        current_b, position_c_request = self.bytes(registers[4])
        position_c, current_c = self.bytes(registers[5])
        position_s_request, position_s = self.bytes(registers[6])
        current_s, _ = self.bytes(registers[7])

        # Update the status:
        self.status.update_gripper_status(gripper_status)
        self.status.update_finger_status(object_status)
        self.status.update_fault_status(fault_status)
        self.status.update_finger_states(
            [
                position_a_request,
                position_a,
                current_a,
                position_b_request,
                position_b,
                current_b,
                position_c_request,
                position_c,
                current_c,
                position_s_request,
                position_s,
                current_s,
            ]
        )

    @staticmethod
    def register(byte1: str, byte2: str) -> int:
        """Combine two bytes into a single register value.

        Args:
            byte1 (str): The first byte as a binary string.
            byte2 (str): The second byte as a binary string.

        Returns:
            int: The combined register value.
        """
        return int(f"0b{byte1}{byte2}", 2)

    @staticmethod
    def bytes(register: int) -> tuple[str, str]:
        """Split a register value into two bytes.

        Args:
            register (int): The register value.

        Returns:
            tuple[str, str]: A tuple containing the two bytes as binary strings.
        """
        binary_str = bin(register)[2:].zfill(16)
        return binary_str[:8], binary_str[8:]

    @staticmethod
    def action(act: str, mod: str, gto: str, atr: str) -> str:
        """Generate the action byte for the gripper command.

        Args:
            act (str): Activation bit (1 = Activate, 0 = Reset).
            mod (str): Mode bits (00 = Basic, 01 = Pinch, 10 = Wide, 11 = Scissor).
            gto (str): Go to bit (1 = Go to requested position, 0 = Stop).
            atr (str): Automatic release bit (1 = Emergency auto-release, 0 = Normal).

        Returns:
            str: The action byte as a binary string.
        """
        return f"{0}{0}{0}{atr}{gto}{mod}{act}"

    @staticmethod
    def options(icf: str, ics: str) -> str:
        """Generate the options byte for the gripper command.

        Args:
            icf (str): Individual control fingers bit (1 = Individual control, 0 = Normal).
            ics (str): Individual control scissor bit (1 = Individual control, 0 = Normal).

        Returns:
            str: The options byte as a binary string.
        """
        return f"{0}{0}{0}{0}{ics}{icf}{0}{0}"

    @staticmethod
    def empty() -> str:
        """Generate an empty byte for the gripper command.

        Returns:
            str: An empty byte as a binary string.
        """
        return f"{0}{0}{0}{0}{0}{0}{0}{0}"

    @staticmethod
    def position(position: int = 0) -> str:
        """Generate the position byte for the gripper command.

        Args:
            position (int): The target position of the gripper (0 = Open, 255 = Closed).

        Returns:
            str: The position byte as a binary string.
        """
        return format(position, "08b")

    @staticmethod
    def speed(speed: int = 0) -> str:
        """Generate the speed byte for the gripper command.

        Args:
            speed (int): The speed of the gripper (0 = 22 mm/s, 255 = 110 mm/s).

        Returns:
            str: The speed byte as a binary string.
        """
        return format(speed, "08b")

    @staticmethod
    def force(force: int = 0) -> str:
        """Generate the force byte for the gripper command.

        Args:
            force (int): The force of the gripper (0 = 15 N, 255 = 60 N).

        Returns:
            str: The force byte as a binary string.
        """
        return format(force, "08b")


def finger_joint_position_from_bit(
    position: int, joint: int, scissor: bool = False
) -> float:
    """Convert a finger position (0-255) to joint angles (rad).

    Args:
        position (int): The position of the finger (0 = Open, 255 = Closed).
        joint (int): The joint number (1, 2, or 3).
        scissor (bool): Whether the finger is a scissor or not.

    Returns:
        float: The joint angle (rad) of the finger.

    Raises:
        ValueError: If scissor is True and joint is 3, as the scissor does not have a third joint.
    """
    match joint:
        case 1:
            lower_limit = -0.1784 if scissor else 0.0
            upper_limit = 0.192 if scissor else 1.2218
        case 2:
            lower_limit = -0.192 if scissor else 0.0
            upper_limit = 0.1784 if scissor else 1.5708
        case 3:
            if scissor:
                raise ValueError("Scissor does not have a third joint.")
            lower_limit = 0.0523
            upper_limit = 1.2217

    reach = upper_limit - lower_limit

    # Only the first joint of the scissor is inverted:
    if scissor and joint == 1:
        angle = upper_limit - (position / 255) * reach
    else:
        angle = lower_limit + (position / 255) * reach

    return angle


@dataclass
class FingerStatus(DataClassJSONMixin):
    """Dataclass to hold the status of a specific finger.

    Attributes:
        scissor (bool): Whether the finger is a scissor or not.
        status (Optional[STATUS_FINGER]): The status of the finger.
        position_request (Optional[int]): The requested position of the finger.
        position (Optional[int]): The position of the finger.
        current (Optional[int]): The current of the finger.
    """

    scissor: bool = False
    status: Optional[STATUS_FINGER] = None
    position_request: Optional[int] = None
    position: Optional[int] = None
    current: Optional[int] = None

    def update_status(self, status: int) -> None:
        """Update the status of the finger.

        Args:
            status (int): The status as an integer.
        """
        self.status = typing.get_args(STATUS_FINGER)[status]

    @property
    def number_of_joints(self) -> int:
        """The number of joints for the finger.

        Returns:
            int: The number of joints (2 for scissor, 3 for regular fingers).
        """
        return 2 if self.scissor else 3


@dataclass
class GripperStatus(DataClassJSONMixin):
    """Dataclass to hold the status of the gripper.

    Attributes:
        active (Optional[bool]): Whether the gripper is active or not.
        mode (Optional[MODES]): The mode of the gripper.
        go_to (Optional[bool]): Whether the gripper can go to the requested position or is stopped.
        gripper_status (Optional[STATUS_GRIPPER]): The general status of the gripper.
        motion_status (Optional[STATUS_MOTION]): The motion status of the gripper.
        fault_status (Optional[STATUS_FAULT]): The fault status of the gripper.
        finger_a (FingerStatus): The status of finger A.
        finger_b (FingerStatus): The status of finger B.
        finger_c (FingerStatus): The status of finger C.
        finger_s (FingerStatus): The status of the scissor.
        fingers (list[FingerStatus]): A list of all fingers.
    """

    # Gripper status:
    active: Optional[bool] = None
    mode: Optional[MODES] = None
    go_to: Optional[bool] = None
    gripper_status: Optional[STATUS_GRIPPER] = None
    motion_status: Optional[STATUS_MOTION] = None

    # Fault status:
    fault_status: Optional[STATUS_FAULT] = None

    # Finger status:
    finger_a: FingerStatus = field(default_factory=FingerStatus)
    finger_b: FingerStatus = field(default_factory=FingerStatus)
    finger_c: FingerStatus = field(default_factory=FingerStatus)
    finger_s: FingerStatus = field(default_factory=FingerStatus)
    fingers: list[FingerStatus] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Initialize the platform configuration."""
        self.finger_s.scissor = True
        self.fingers = [self.finger_a, self.finger_b, self.finger_c, self.finger_s]

    def update_gripper_status(self, gripper_status: str) -> None:
        """Update the general status of the gripper.

        Args:
            gripper_status (str): The general status as a binary string (8 bits).
        """
        byte = gripper_status[::-1]
        self.active = bool(byte[0])
        self.mode = typing.get_args(MODES)[int(byte[1:3], 2)]
        self.go_to = bool(byte[3])
        self.gripper_status = typing.get_args(STATUS_GRIPPER)[int(byte[4:6], 2)]
        self.motion_status = typing.get_args(STATUS_MOTION)[int(byte[6:8], 2)]

    def update_finger_status(self, object_status: str) -> None:
        """Update the status of a specific finger.

        Args:
            object_status (str): The object status as a binary string (8 bits).
        """
        byte = object_status[::-1]
        self.finger_a.update_status(int(byte[0:2], 2))
        self.finger_b.update_status(int(byte[2:4], 2))
        self.finger_c.update_status(int(byte[4:6], 2))
        self.finger_s.update_status(int(byte[6:8], 2))

    def update_fault_status(self, fault_status: str) -> None:
        """Update the fault status of the gripper.

        Args:
            fault_status (str): The fault status as a binary string (8 bits).
        """
        byte = fault_status[::-1]
        fault_status_int = int(byte[0:4], 2)
        match fault_status_int:
            case 0:
                self.fault_status = typing.get_args(STATUS_FAULT)[0]
            case 5:
                self.fault_status = typing.get_args(STATUS_FAULT)[1]
            case 6:
                self.fault_status = typing.get_args(STATUS_FAULT)[2]
            case 7:
                self.fault_status = typing.get_args(STATUS_FAULT)[3]
            case 9:
                self.fault_status = typing.get_args(STATUS_FAULT)[4]
            case 10:
                self.fault_status = typing.get_args(STATUS_FAULT)[5]
            case 11:
                self.fault_status = typing.get_args(STATUS_FAULT)[6]
            case 12:
                self.fault_status = typing.get_args(STATUS_FAULT)[7]
            case 13:
                self.fault_status = typing.get_args(STATUS_FAULT)[8]
            case 14:
                self.fault_status = typing.get_args(STATUS_FAULT)[9]

    def update_finger_states(self, states: list[str]) -> None:
        """Update the status of the fingers.

        Args:
            states (list[str]): A list of binary strings representing the states of the fingers.
        """
        fingers = [self.finger_a, self.finger_b, self.finger_c, self.finger_s]

        for n in range(len(fingers)):
            finger = fingers[n]
            finger.position_request = int(states[n * 3], 2)
            finger.position = int(states[n * 3 + 1], 2)
            finger.current = int(states[n * 3 + 2], 2)


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
