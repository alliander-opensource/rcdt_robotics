# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import time

import launch_pytest
import pytest
import rclpy
import rclpy.node
from builtin_interfaces.msg import Time as TimeMsg
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from rcdt_utilities.launch_utils import (
    assert_for_message,
    assert_for_nodes,
    get_file_path,
)
from rcdt_utilities.test_utils import (
    call_move_to_configuration_service,
    call_trigger_service,
    get_joint_position,
)
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Header


@launch_pytest.fixture(scope="module")
def franka_launch() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                get_file_path("rcdt_franka", ["launch"], "franka.launch.py"),
                launch_arguments={
                    "rviz": "False",
                    "world": "empty_camera.sdf",
                    "realsense": "True",
                }.items(),
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=franka_launch)
def test_joint_states_published() -> None:
    """Test that joint states are published. This is a basic test to check that the
    launch file is working and that the robot is publishing joint states."""
    assert_for_message(JointState, "/joint_states", 60)


@pytest.mark.launch(fixture=franka_launch)
def test_moveit_node_activated() -> None:
    """Test that joint states are published. This is a basic test to check that the
    launch file is working and that the robot is publishing joint states."""
    assert_for_nodes(["move_group", "moveit_manager"], 30)


@pytest.mark.parametrize(
    "service, expected_value",
    [
        ("/close_gripper", 0.00),
        ("/open_gripper", 0.08),
    ],
)
@pytest.mark.launch(fixture=franka_launch)
def test_gripper_action(
    service: str, expected_value: float, singleton_node: rclpy.node.Node
) -> None:
    """Test gripper open/close action and verify joint state."""
    assert call_trigger_service(singleton_node, service) is True

    joint_value = get_joint_position("fr3_finger_joint1")
    assert joint_value * 2 == pytest.approx(expected_value, abs=3e-3)


@pytest.mark.launch(fixture=franka_launch)
def test_move_to_drop_configuration(singleton_node: rclpy.node.Node) -> None:
    """Test that MoveIt can move to a configuration."""

    assert call_move_to_configuration_service(singleton_node, "drop") is True
    drop_values = [-1.57079632679, -0.65, 0, -2.4, 0, 1.75, 0.78539816339]
    for i in range(7):
        assert get_joint_position(f"fr3_joint{i + 1}") == pytest.approx(
            drop_values[i], abs=0.01
        )


@pytest.mark.launch(fixture=franka_launch)
@pytest.mark.parametrize(
    "buttons, expected_value",
    [
        ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], None),
        ([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], 0.08),
        ([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1], 0.0),
    ],
)
def test_joy_gripper_node(
    buttons: list[int], expected_value: float, singleton_node: rclpy.node.Node
) -> None:
    """Test that the joy node is running and the gripper is moving.
    The first call is an initialization call.
    The second call is a call to open the gripper.
    The third call is a call to close the gripper.
    """

    pub = singleton_node.create_publisher(Joy, "/joy", 10)

    msg = Joy()
    msg.header = Header(stamp=TimeMsg(sec=0, nanosec=0), frame_id="")
    msg.axes = [0.0] * 5
    msg.buttons = buttons

    pub.publish(msg)
    rclpy.spin_once(singleton_node, timeout_sec=0.1)

    if expected_value is not None:
        # Wait for the gripper to move
        time.sleep(1.0)
        # Check the gripper position
        joint_value = get_joint_position("fr3_finger_joint1")
        assert joint_value * 2 == pytest.approx(expected_value, abs=2.5e-3)
