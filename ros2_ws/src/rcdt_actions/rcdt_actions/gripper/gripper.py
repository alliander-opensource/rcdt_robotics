from rcdt_actions.definitions import Action
from std_srvs.srv import Trigger


def open_gripper() -> Action:
    return Action("/open_gripper", Trigger)


def close_gripper() -> Action:
    return Action("/close_gripper", Trigger)
