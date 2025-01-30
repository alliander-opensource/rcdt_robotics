from rcdt_actions.definitions import Action
from rcdt_utilities_msgs.srv import MoveToConfiguration, MoveHandToPose

ns = "/moveit_manager"


def move_to_configuration() -> Action:
    return Action(f"{ns}/move_to_configuration", MoveToConfiguration)


def move_hand_to_pose() -> Action:
    return Action(f"{ns}/move_hand_to_pose", MoveHandToPose)
