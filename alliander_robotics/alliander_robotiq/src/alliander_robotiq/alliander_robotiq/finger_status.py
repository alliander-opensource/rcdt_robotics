# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import typing
from dataclasses import dataclass
from typing import Literal, Optional

from mashumaro.mixins.json import DataClassJSONMixin

STATUS_FINGER = Literal[
    "in_motion",
    "contact_while_opening",
    "contact_while_closing",
    "reached_target_position",
]


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
