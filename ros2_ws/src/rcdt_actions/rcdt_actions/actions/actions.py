# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from rcdt_actions.definitions import Sequence
from rcdt_actions import gripper
from rcdt_actions import moveit

Z_ABOVE = 0.15
Z_PICK = 0.07

above = Sequence(
    "above",
    [
        gripper.open_gripper(),
        moveit.tranform_goal_pose().set_args({"axis": "z", "value": Z_ABOVE}),
        moveit.move_hand_to_pose(),
    ],
)

pick = Sequence(
    "pick",
    [
        gripper.open_gripper(),
        moveit.tranform_goal_pose().set_args({"axis": "z", "value": Z_ABOVE}),
        moveit.move_hand_to_pose(),
        moveit.tranform_goal_pose().set_args({"axis": "z", "value": -Z_PICK}),
        moveit.move_hand_to_pose().set_args({"planning_type": "LIN"}),
        gripper.close_gripper(),
        moveit.tranform_goal_pose().set_args({"axis": "z", "value": Z_PICK}),
        moveit.move_hand_to_pose().set_args({"planning_type": "LIN"}),
        moveit.move_to_configuration().set_args({"configuration": "home"}),
    ],
)

drop = Sequence(
    "drop",
    [
        moveit.move_to_configuration().set_args({"configuration": "drop"}),
        gripper.open_gripper(),
        gripper.close_gripper(),
        moveit.move_to_configuration().set_args({"configuration": "home"}),
    ],
)
