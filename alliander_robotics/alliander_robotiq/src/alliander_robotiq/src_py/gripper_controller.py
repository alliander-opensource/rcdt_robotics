#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

# Based on: https://github.com/baha2r/robotiq3f_py/blob/main/robotiqcontrol/GripperController.py

import json
import threading
import time
import typing
import warnings

import rclpy
from alliander_interfaces.action import TriggerAction
from pyModbusTCP.client import ModbusClient
from rclpy.action.server import ActionServer, ServerGoalHandle
from rclpy.node import Node
from std_msgs.msg import String

IP = "127.0.0.1"
RATE = 0.1

MODES = typing.Literal[
    "Basic",
    "Pinch",
    "Wide",
    "Scissor",
]


class GripperController(Node):
    """Node to control the Robotiq 3-finger gripper."""

    def __init__(self) -> None:
        """Initialize node."""
        super().__init__("gripper_controller")

        # Initialize Modbus client and update thread
        self.modbus_client = ModbusClient(IP, timeout=5, auto_open=False)

        self.get_logger().info("Connecting with gripper...")

        connected = self.modbus_client.open()
        if connected:
            self.get_logger().info("Connection successful.")
        else:
            self.get_logger().error("Connection failed.")

        # ROS:
        self.status = {
            "FaultStatus": 0,
            "FingerA_Pos": 0,
            "FingerA_Cur": 0,
            "FingerB_Pos": 0,
            "FingerB_Cur": 0,
            "FingerC_Pos": 0,
            "FingerC_Cur": 0,
            "Scissor_Pos": 0,
            "Scissor_Cur": 0,
        }
        self.publisher = self.create_publisher(String, "~/gripper_status", 10)
        # self.timer = self.create_timer(RATE, self.update_status)

        self.activate()
        self.get_logger().info("Gripper initialized.")
        time.sleep(30)
        self.get_logger().info("Closing gripper...")
        self.command_gripper(position=10, speed=250, force=250)
        self.get_logger().info("Closed.")

    def update_status(self):
        data = self.modbus_client.read_input_registers(0, 8)
        if not data:
            self.get_logger().error("Error reading data in status.")
            return

        self.status["FaultStatus"], self.FingerA_PosReqEcho = self.byte_status(
            self.add_leading_zeros(bin(data[1]))
        )
        self.status["FingerA_Pos"], self.status["FingerA_Cur"] = self.byte_status(
            self.add_leading_zeros(bin(data[2]))
        )
        self.FingerB_PositionReqEcho, self.status["FingerB_Pos"] = self.byte_status(
            self.add_leading_zeros(bin(data[3]))
        )
        self.status["FingerB_Cur"], self.FingerC_PositionReqEcho = self.byte_status(
            self.add_leading_zeros(bin(data[4]))
        )
        self.status["FingerC_Pos"], self.status["FingerC_Cur"] = self.byte_status(
            self.add_leading_zeros(bin(data[5]))
        )
        self.Scissor_PositionReqEcho, self.status["Scissor_Pos"] = self.byte_status(
            self.add_leading_zeros(bin(data[6]))
        )
        self.status["Scissor_Cur"], RES = self.byte_status(
            self.add_leading_zeros(bin(data[7]))
        )

        msg = String()
        msg.data = json.dumps(self.status)
        self.publisher.publish(msg)

    def action_callback(self, goal_handle: ServerGoalHandle) -> TriggerAction.Result:
        """Execute the action server callback.

        Args:
            goal_handle (ServerGoalHandle): The goal handle for the action server.

        Returns:
            TriggerAction.Result: The result of the action server execution.
        """
        result = TriggerAction.Result()
        goal_handle.succeed()
        result.success = True
        return result

    def activate(self) -> None:
        self.command_gripper()

    def command_gripper(
        self,
        mode: MODES = "Basic",
        position: int = 0,  # 0 = Open | 255 = Closed
        speed: int = 0,  # 0 = 22 mm/s | 255 = 110 mm/s
        force: int = 0,  # 0 = 15 N | 255 = 60 N
    ):
        act = 1  # Activate | When setting rACT to one, the Gripper will begin movement to complete its auto-calibration feature | rACT bit must remain on afterwards for any other action to be performed.
        gto = 1  # Go To
        icf = 0  # Individual Finger Control

        mod = list(typing.get_args(MODES)).index(mode)  # Mode

        self.modbus_client.write_multiple_registers(
            0,
            [
                self._action_req_variable(rACT=act, rGTO=gto, rMOD=mod, rICF=icf),
                self._position_req_variable(position),
                self._write_req_variable(speed, force),
            ],
        )

    def command_individual_fingers(
        self,
        rPRA=[1, 1, 1],
        rSP=[250, 250, 250],
        rFR=[250, 250, 250],
        rMOD="Basic",
        rICF=False,
    ):
        """Send a command to the gripper."""
        # self.status()
        modes = {"Basic": 0, "Pinch": 1, "Wide": 2, "Scissor": 3}
        rMOD = modes[rMOD]
        if rICF:
            for var in [rPRA, rSP, rFR]:
                if isinstance(var, int):
                    self.close()
                    raise ValueError(
                        "Input variables must be 3d vectors when using Individual Control Flag."
                    )
            response = self.modbus_client.write_multiple_registers(
                0,
                [
                    self._action_req_variable(rACT=1, rGTO=1, rMOD=rMOD, rICF=1),
                    self._position_req_variable(rPRA[0]),
                    self._write_req_variable(rSP[0], rFR[0]),
                    self._write_req_variable(rPRA[1], rSP[1]),
                    self._write_req_variable(rFR[1], rPRA[2]),
                    self._write_req_variable(rSP[2], rFR[2]),
                ],
            )
        else:
            for var in [rPRA, rSP, rFR]:
                if isinstance(var, list):
                    warnings.warn(
                        "only first value of 3d vector will be used when not using Individual Control Flag."
                    )
            response = self.modbus_client.write_multiple_registers(
                0,
                [
                    self._action_req_variable(rACT=1, rGTO=1, rMOD=rMOD, rICF=0),
                    self._position_req_variable(rPRA[0]),
                    self._write_req_variable(rSP[0], rFR[0]),
                ],
            )

    @staticmethod
    def add_leading_zeros(bin_num: str, total_length: int = 16) -> str:
        """Ensure binary number string has correct number of digits.

        Args:
            bin_num (str): binary number
            total_length (int): length of the binary number

        Returns:
            str: binary number
        """
        bin_str = str(bin_num)[2:]  # remove '0b' prefix
        return bin_str.zfill(total_length)

    @staticmethod
    def byte_status(variable: str) -> tuple[int, int]:
        """Split and parse byte status.

        Args:
            variable (str): binary number

        Returns:
            tuple[int, int]: parsed byte values
        """
        B1 = int(variable[0:8], 2)
        B2 = int(variable[8:16], 2)
        return B1, B2

    @staticmethod
    def stat(variable: str) -> list:
        """Split and parse status.

        Args:
            variable (str): binary number

        Returns:
            list: parsed status values
        """
        # Define gripper modes
        modes = ["Basic Mode", "Pinch Mode", "Wide Mode", "Scissor Mode"]

        # Split and parse status bits
        gSTA = int(variable[0:2], 2)
        gIMC = int(variable[2:4], 2)
        gGTO = int(variable[4], 2)
        gMOD = int(variable[5:7], 2)
        gMOD = modes[gMOD]  # Translate mode code to string
        gACT = int(variable[7], 2)
        gDTS = int(variable[8:10], 2)
        gDTC = int(variable[10:12], 2)
        gDTB = int(variable[12:14], 2)
        gDTA = int(variable[14:16], 2)

        return [gSTA, gIMC, gGTO, gMOD, gACT, gDTS, gDTC, gDTB, gDTA]

    def activate(self) -> None:
        """Activate the gripper."""
        self.modbus_client.write_multiple_registers(
            0,
            [
                self._action_req_variable(rACT=1, rGTO=1, rMOD=0, rICF=0),
                self._position_req_variable(0),
                self._write_req_variable(0, 0),
            ],
        )
        print("Gripper activate")
        time.sleep(1)

    def _action_req_variable(
        self,
        rARD: int = 0,
        rATR: int = 0,
        rGTO: int = 0,
        rACT: int = 0,
        rMOD: int = 0,
        rICS: int = 0,
        rICF: int = 0,
    ) -> int:
        """Build action request variable."""
        # Check if the input variables are either 0 or 1
        for var in [rARD, rATR, rGTO, rACT]:
            if var not in {0, 1}:
                raise ValueError("Input variables must be either 0 or 1.")
        rMOD = int(bin(rMOD).replace("0b", "").zfill(2))
        # Construct the string variable
        string_variable = f"0b00{rARD}{rATR}{rGTO}{rMOD}{rACT}0000{rICS}{rICF}00"

        return int(string_variable, 2)

    def _position_req_variable(self, rPR: int = 0) -> int:
        """Build position request variable."""
        # Check if the input variables are between 0 or 255
        for var in [rPR]:
            if var not in range(0, 256):
                raise ValueError("Input variables must be between 0 and 255.")
        rPR = int(format(rPR, "08b"))

        # Construct the string variable
        string_variable = f"0b00000000{rPR}"

        return int(string_variable, 2)

    def _write_req_variable(self, X: int = 0, Y: int = 0) -> int:
        """Build write request variable."""
        # Check if the input variables are between 0 to 255
        for var in [X, Y]:
            if var not in range(0, 256):
                raise ValueError("Input variables must be between 0 and 255.")
        X = int(format(X, "08b"))
        Y = int(format(Y, "08b"))
        # Construct the string variable
        string_variable = f"0b{X}{Y}"

        return int(string_variable, 2)

    def close(self) -> None:
        """Stop the update thread and close the Modbus client."""
        self._running = False
        self._thread.join()
        self.modbus_client.close()
        print("Connection closed.")


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and set the thresholds.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    griper_controller = GripperController()
    rclpy.spin(griper_controller)
    griper_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
