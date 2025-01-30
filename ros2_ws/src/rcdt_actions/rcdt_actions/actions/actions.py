from rcdt_actions.definitions import Sequence
from rcdt_actions.gripper import gripper

pick = Sequence(
    "pick",
    [
        gripper.open_gripper,
        gripper.close_gripper,
    ],
)
