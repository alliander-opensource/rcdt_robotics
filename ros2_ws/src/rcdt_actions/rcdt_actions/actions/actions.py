from rcdt_actions.definitions import Sequence
from rcdt_actions.gripper import gripper
from rcdt_actions.moveit import moveit
from rcdt_actions.detection import detection

pick = Sequence(
    "pick",
    [
        detection.get_rgbd_from_topic().set_args({"topic": "/camera/camera/rgbd"}),
        detection.segment_image().set_links({"rgb_image": "input_image"}),
        gripper.open_gripper(),
        moveit.move_to_configuration().set_args({"configuration": "drop"}),
        gripper.close_gripper(),
        moveit.move_to_configuration().set_args({"configuration": "home"}),
    ],
)
