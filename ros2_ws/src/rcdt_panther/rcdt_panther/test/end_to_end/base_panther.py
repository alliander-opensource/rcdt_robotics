# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from geometry_msgs.msg import Pose
from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.test_utils import (
    assert_joy_topic_switch,
    assert_movements_with_joy,
    call_trigger_service,
    wait_for_register,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState


def get_tests() -> dict:
    """Test class for the Panther."""

    def test_wait_for_register(_self: object, timeout: int) -> None:
        wait_for_register(timeout=timeout)

    def test_joint_states_published(_self: object, timeout: int) -> None:
        assert_for_message(JointState, "/panther/joint_states", timeout=timeout)

    def test_e_stop_request(_self: object, test_node: Node, timeout: int) -> None:
        assert (
            call_trigger_service(
                node=test_node,
                service_name="/panther/hardware/e_stop_reset",
                timeout=timeout,
            )
            is True
        )

    def test_switch_joy_to_panther_topic(
        _self: object, test_node: Node, timeout: int
    ) -> None:
        assert_joy_topic_switch(
            node=test_node,
            expected_topic="/panther/joy",
            button_config=[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            timeout=timeout,
        )

    def test_move_panther_with_joy(
        _self: object, test_node: Node, timeout: int
    ) -> None:
        def compare_fn(p1: Pose, p2: Pose) -> float:
            return p2.position.x - p1.position.x

        assert_movements_with_joy(
            node=test_node,
            joy_axes=[0.0, 1.0, 0.0, 0.0, 0.0],
            compare_fn=compare_fn,
            threshold=0.1,
            description="x position",
            frame_base="panther/base_link",
            frame_target="panther/odom",
            timeout=timeout,
        )

    def test_rotate_panther_with_joy(
        _self: object, test_node: Node, timeout: int
    ) -> None:
        def compare_fn(p1: Pose, p2: Pose) -> float:
            return p2.orientation.w - p1.orientation.w

        assert_movements_with_joy(
            node=test_node,
            joy_axes=[0.0, 0.0, 1.0, 0.0, 0.0],
            compare_fn=compare_fn,
            threshold=0.01,
            description="orientation.w",
            frame_base="panther/base_link",
            frame_target="panther/odom",
            timeout=timeout,
        )

        # Collect all test methods defined above

    tests = {
        name: obj
        for name, obj in locals().items()
        if callable(obj) and name.startswith("test_")
    }
    return tests
