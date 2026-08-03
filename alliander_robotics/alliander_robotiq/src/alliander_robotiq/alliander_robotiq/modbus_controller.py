# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import typing

from pyModbusTCP.client import ModbusClient
from rclpy.publisher import Publisher
from std_msgs.msg import Float64MultiArray

from alliander_robotiq.gripper_status import MODES, GripperStatus


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
    # Upper and lower limits are obtained from the URDF.
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
