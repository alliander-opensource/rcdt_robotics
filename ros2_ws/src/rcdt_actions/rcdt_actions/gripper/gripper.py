# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from std_srvs.srv import Trigger

from rcdt_actions.definitions import Action


def open_gripper() -> Action:
    """Create an action to open the gripper.

    Returns:
        Action: An action that can be used to open the gripper.

    """
    return Action("/open_gripper", Trigger)


def close_gripper() -> Action:
    """Create an action to close the gripper.

    Returns:
        Action: An action that can be used to close the gripper.

    """
    return Action("/close_gripper", Trigger)
