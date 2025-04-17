import launch_pytest
import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_utilities.launch_utils import assert_for_message, get_file_path
from sensor_msgs.msg import JointState


@launch_pytest.fixture(scope="module")
def launch_description() -> LaunchDescription:
    franka = IncludeLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "franka.launch.py")
    )

    return LaunchDescription(
        [
            franka,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=launch_description)
def test_joint_states_published() -> None:
    assert_for_message(JointState, "/joint_states", 10)
