# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import launch_pytest
import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_utilities.launch_utils import (
    assert_for_message,
)
from rcdt_utilities.test_utils import (
    get_joint_position,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from utils import call_move_gripper_service, follow_joint_trajectory_goal


@launch_pytest.fixture(scope="module")
def franka_core_launch(
    core_launch: IncludeLaunchDescription, controllers_launch: IncludeLaunchDescription
) -> LaunchDescription:
    return LaunchDescription(
        [
            core_launch,
            controllers_launch,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=franka_core_launch)
def test_joint_states_published() -> None:
    assert_for_message(JointState, "franka/joint_states", 60)


@pytest.mark.launch(fixture=franka_core_launch)
def test_can_move_fingers(singleton_node: Node) -> None:
    assert (
        call_move_gripper_service(
            singleton_node,
            width=0.04,
            action_name="/franka/gripper_action_controller/gripper_cmd",
        )
        is True
    )
    pos = get_joint_position(name_space="franka", joint="fr3_finger_joint1") * 2
    assert pos == pytest.approx(0.08, abs=0.005), f"Got gripper position of {pos}"


@pytest.mark.launch(fixture=franka_core_launch)
def test_follow_joint_trajectory_goal(singleton_node: Node) -> None:
    follow_joint_trajectory_goal(
        singleton_node,
        positions=[0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.0],
        controller="franka/fr3_arm_controller",
    )
    pos_joint2 = get_joint_position(name_space="franka", joint="fr3_joint2")
    pos_joint3 = get_joint_position(name_space="franka", joint="fr3_joint3")

    assert pos_joint2 == pytest.approx(-0.5, abs=0.01), (
        f"Got joint position {pos_joint2}"
    )
    assert pos_joint3 == pytest.approx(0.5, abs=0.01), (
        f"Got joint position {pos_joint3}"
    )
