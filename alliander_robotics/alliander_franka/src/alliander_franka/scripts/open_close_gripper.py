#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
import sys
from enum import Enum, auto

import rclpy
from alliander_interfaces.action import TriggerAction
from rclpy.action import ActionClient

ACTION_PREFIX: str = "/franka/gripper"


class GripperPosition(Enum):
    """Enum containing different gripper positions.

    Attributes:
        OPEN: Open the gripper.
        CLOSE: Close the gripper.
    """

    OPEN = auto()
    CLOSE = auto()


if __name__ == "__main__":
    command = sys.argv[1] if len(sys.argv) > 1 else "open"

    try:
        gripper_position = GripperPosition[command.upper()]
    except KeyError:
        print(
            f"ERROR: Invalid GripperPosition {command}. Options are {[p.value for p in GripperPosition]}."
        )
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node("gripper_position_helper")

    if gripper_position.value == "OPEN":
        ac = ActionClient(node, TriggerAction, f"{ACTION_PREFIX}/open")
    else:
        ac = ActionClient(node, TriggerAction, f"{ACTION_PREFIX}/close")

    future = ac.send_goal_async(TriggerAction.Goal())
    rclpy.spin_until_future_complete(node, future)
