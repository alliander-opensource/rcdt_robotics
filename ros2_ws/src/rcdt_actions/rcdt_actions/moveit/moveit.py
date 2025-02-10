# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from rcdt_actions.definitions import Action
from rcdt_utilities_msgs.srv import (
    MoveToConfiguration,
    MoveHandToPose,
    TransformGoalPose,
)

ns = "/moveit_manager"


def tranform_goal_pose() -> Action:
    return Action(f"{ns}/transform_goal_pose", TransformGoalPose)


def move_to_configuration() -> Action:
    return Action(f"{ns}/move_to_configuration", MoveToConfiguration)


def move_hand_to_pose() -> Action:
    return Action(f"{ns}/move_hand_to_pose", MoveHandToPose)
