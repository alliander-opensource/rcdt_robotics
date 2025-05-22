# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import pytest
import rclpy
from rcdt_utilities.launch_utils import assert_for_message, assert_for_node
from rcdt_utilities.test_utils import (
    assert_joy_topic_switch,
    assert_movements_with_joy,
    wait_until_active,
    wait_until_reached_joint,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy


def get_tests() -> dict:
    """Test class for the Franka robot."""

    def test_joint_states_published(_self: object) -> None:
        """Test that joint states are published. This is a basic test to check that the
        launch file is working and that the robot is publishing joint states."""
        assert_for_message(JointState, "franka/joint_states", 60)

    def test_ready_to_start(_self: object, test_node: Node) -> None:
        """This test will ensure the tests are ready to start by waiting for the move_group and moveit_manager node.
        Also waits until the gripper_action_controller is active."""
        assert (
            assert_for_node(
                fully_qualified_node_name="franka/move_group",
                test_node=test_node,
                timeout=30,
            )
            is True
        )
        assert (
            assert_for_node(
                fully_qualified_node_name="franka/moveit_manager",
                test_node=test_node,
                timeout=30,
            )
            is True
        )
        assert (
            wait_until_active(
                node=test_node,
                controller_name="gripper_action_controller",
                controller_manager_name="/franka/controller_manager",
            )
            is True
        )

    def test_switch_joy_to_franka_topic(_self: object, test_node: Node) -> None:
        """Test to see if the switch to Franka mode is correct."""
        assert_joy_topic_switch(
            node=test_node,
            expected_topic="/franka/joy",
            button_config=[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        )

    @pytest.mark.parametrize(
        "buttons, expected_value",
        [
            ([1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], 0.04),
            ([1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1], 0.0),
        ],
    )
    def test_joy_gripper_node(
        _self: object,
        buttons: list[int],
        expected_value: float,
        test_node: Node,
        finger_joint_fault_tolerance: float,
    ) -> None:
        """Test that the joy node is running and the gripper is moving.
        The first call is an initialization call.
        The second call is a call to open the gripper.
        The third call is a call to close the gripper.
        """

        pub = test_node.create_publisher(Joy, "/joy", 10)

        msg = Joy()
        msg.buttons = buttons
        pub.publish(msg)
        rclpy.spin_once(test_node, timeout_sec=0.1)

        reached_goal, joint_value = wait_until_reached_joint(
            namespace="franka",
            joint="fr3_finger_joint1",
            expected_value=expected_value,
            tolerance=finger_joint_fault_tolerance,
        )
        assert reached_goal is True, (
            f"The joint did not reach the joint. Currently {joint_value}, expected {expected_value}"
        )

    @pytest.mark.parametrize(
        "axes, direction",
        [
            ([1.0, 0.0, 0.0, 0.0, 0.0], "y"),
            ([0.0, 1.0, 0.0, 0.0, 0.0], "x"),
            ([0.0, 0.0, 0.0, 1.0, 0.0], "z"),
        ],
    )
    def test_move_arm_with_joy(
        _self: object,
        axes: list[float],
        direction: str,
        test_node: Node,
        movement_threshold: float = 0.01,
    ) -> None:
        """Tests the linear movements of the hand while controlling with the joystick.
        Assert if it moves above a certain movement_threshold."""
        assert_movements_with_joy(
            node=test_node,
            joy_axes=axes,
            compare_fn=lambda p1, p2: getattr(p2.position, direction)
            - getattr(p1.position, direction),
            threshold=movement_threshold,
            description=f"{direction} position",
            frame_base="franka/fr3_hand",
            frame_target="franka/fr3_link0",
        )

    # Collect all test methods defined above
    tests = {
        name: obj
        for name, obj in locals().items()
        if callable(obj) and name.startswith("test_")
    }
    return tests


def FrankaTestSuite() -> object:  # noqa: N802
    """Dynamically create a test class with the test methods."""
    return type("FrankaFullTests", (), get_tests())
