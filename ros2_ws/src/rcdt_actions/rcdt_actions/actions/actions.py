from rcdt_actions.definitions import Sequence
from rcdt_actions.gripper import gripper
from rcdt_actions.moveit import moveit

pick = Sequence(
    "pick",
    [
        gripper.open_gripper(),
        moveit.tranform_goal_pose().set_args({"axis": "z", "value": 0.15}),
        moveit.move_hand_to_pose(),
        moveit.tranform_goal_pose().set_args({"axis": "z", "value": -0.07}),
        moveit.move_hand_to_pose().set_args({"planning_type": "LIN"}),
        gripper.close_gripper(),
        moveit.tranform_goal_pose().set_args({"axis": "z", "value": 0.07}),
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
