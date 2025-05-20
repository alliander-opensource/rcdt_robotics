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


class PantherFullTests:
    def test_joint_states_published(self) -> None:
        assert_for_message(JointState, "/panther/joint_states", 60)

    def test_ready_to_start(self, test_node: Node) -> None:
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

    def test_e_stop_request(self, test_node: Node) -> None:
        assert (
            call_trigger_service(
                node=test_node, service_name="/panther/hardware/e_stop_reset"
            )
            is True
        )

    def test_switch_joy_to_panther_topic(self, test_node: Node) -> None:
        assert_joy_topic_switch(
            node=test_node,
            expected_topic="/panther/joy",
            button_config=[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        )

    def test_move_panther_with_joy(self, test_node: Node) -> None:
        assert_movements_with_joy(
            node=test_node,
            joy_axes=[0.0, 1.0, 0.0, 0.0, 0.0],
            compare_fn=lambda p1, p2: p2.position.x - p1.position.x,
            threshold=0.1,
            description="x position",
            frame_base="panther/base_link",
            frame_target="panther/odom"
            )

    def test_rotate_panther_with_joy(self, test_node: Node) -> None:
        assert_movements_with_joy(
            node=test_node,
            joy_axes=[0.0, 0.0, 1.0, 0.0, 0.0],
            compare_fn=lambda p1, p2: p2.orientation.w - p1.orientation.w,
            threshold=0.01,
            description="orientation.w",
            frame_base="panther/base_link",
            frame_target="panther/odom"
        )
