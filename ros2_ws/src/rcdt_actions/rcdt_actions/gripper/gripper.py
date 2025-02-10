# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from std_srvs.srv import Trigger

from rcdt_actions.definitions import Action


def open_gripper() -> Action:
    return Action("/open_gripper", Trigger)


def close_gripper() -> Action:
    return Action("/close_gripper", Trigger)
