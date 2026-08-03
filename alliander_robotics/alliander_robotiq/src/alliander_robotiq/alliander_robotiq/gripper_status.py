# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import typing
from dataclasses import dataclass, field
from typing import Literal, Optional

from mashumaro.mixins.json import DataClassJSONMixin

from alliander_robotiq.finger_status import FingerStatus

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

    @property
    def fingers(self) -> list[FingerStatus]:
        """A list of all fingers."""
        return [self.finger_a, self.finger_b, self.finger_c, self.finger_s]

    def __post_init__(self) -> None:
        """Initialize the platform configuration."""
        self.finger_s.scissor = True

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
