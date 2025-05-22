# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.test_utils import (
    assert_joy_topic_switch,
    assert_movements_with_joy,
    call_trigger_service,
    wait_until_active,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState


def get_tests() -> dict:
    """Test class for the Panther."""

    def test_joint_states_published(_self: object) -> None:
        """Test that joint states are published. This is a basic test to check that the
        launch file is working and that the robot is publishing joint states."""
        assert_for_message(JointState, "/panther/joint_states", 60)

    def test_ready_to_start(_self: object, test_node: Node) -> None:
        """This test will ensure the tests are ready to start by waiting for the move_group and moveit_manager node.
        Also waits until the gripper_action_controller is active."""

        assert (
            wait_until_active(
                node=test_node,
                controller_name="drive_controller",
                controller_manager_name="/panther/controller_manager",
            )
            is True
        )

    def test_e_stop_request(_self: object, test_node: Node) -> None:
        """Test to see if the e-stop request is correct."""
        assert (
            call_trigger_service(
                node=test_node, service_name="/panther/hardware/e_stop_reset"
            )
            is True
        )

    def test_switch_joy_to_panther_topic(_self: object, test_node: Node) -> None:
        """Test to see if the switch to Panther mode is correct."""
        assert_joy_topic_switch(
            node=test_node,
            expected_topic="/panther/joy",
            button_config=[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        )

    def test_move_panther_with_joy(_self: object, test_node: Node) -> None:
        """Test to see if the panther moves with the joy axes."""
        assert_movements_with_joy(
            node=test_node,
            joy_axes=[0.0, 1.0, 0.0, 0.0, 0.0],
            compare_fn=lambda p1, p2: p2.position.x - p1.position.x,
            threshold=0.1,
            description="x position",
            frame_base="panther/base_link",
            frame_target="panther/odom",
        )

    def test_rotate_panther_with_joy(_self: object, test_node: Node) -> None:
        """Test to see if the panther rotates with the joy axes."""
        assert_movements_with_joy(
            node=test_node,
            joy_axes=[0.0, 0.0, 1.0, 0.0, 0.0],
            compare_fn=lambda p1, p2: p2.orientation.w - p1.orientation.w,
            threshold=0.01,
            description="orientation.w",
            frame_base="panther/base_link",
            frame_target="panther/odom",
        )

        # Collect all test methods defined above

    tests = {
        name: obj
        for name, obj in locals().items()
        if callable(obj) and name.startswith("test_")
    }
    return tests


def PantherTestSuite() -> object:  # noqa: N802
    """Dynamically create a test class with the test methods."""
    return type("PantherFullTests", (), get_tests())
