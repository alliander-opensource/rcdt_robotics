# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from rcdt_messages.srv import MoveHandToPose, MoveToConfiguration, TransformGoalPose

from rcdt_actions.definitions import Action

ns = "/moveit_manager"


def tranform_goal_pose() -> Action:
    """Create an action to transform a goal pose.

    Returns:
        Action: An action that can be used to transform a goal pose.

    """
    return Action(f"{ns}/transform_goal_pose", TransformGoalPose)


def move_to_configuration() -> Action:
    """Create an action to move to a configuration.

    Returns:
        Action: An action that can be used to move to a configuration.

    """
    return Action(f"{ns}/move_to_configuration", MoveToConfiguration)


def move_hand_to_pose() -> Action:
    """Create an action to move the hand to a pose.

    Returns:
        Action: An action that can be used to move the hand to a pose.

    """
    return Action(f"{ns}/move_hand_to_pose", MoveHandToPose)
