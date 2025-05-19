# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.test_utils import call_trigger_service, assert_joy_topic_switch
from rclpy.node import Node
from sensor_msgs.msg import JointState
from utils import EndToEndUtils, call_express_pose_in_other_frame
import pytest
from geometry_msgs.msg import PoseStamped
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rcdt_utilities.launch_utils import assert_for_message, assert_for_node
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from utils import EndToEndUtils, call_express_pose_in_other_frame


class PantherFullTests(EndToEndUtils):
    def test_joint_states_published(self) -> None:
        assert_for_message(JointState, "/panther/joint_states", 60)

    def test_ready_to_start(self, singleton_node: Node) -> None:
        """This test will ensure the tests are ready to start by waiting for the move_group and moveit_manager node.
        Also waits until the gripper_action_controller is active."""

        assert (
            EndToEndUtils().wait_until_active(
                node=singleton_node,
                controller_name="drive_controller",
                controller_manager_name="/panther/controller_manager",
            )
            is True
        )

    def test_e_stop_request(self, singleton_node: Node) -> None:
        assert (
            call_trigger_service(
                node=singleton_node, service_name="/panther/hardware/e_stop_reset"
            )
            is True
        )

    def test_switch_joy_to_panther_topic(self, singleton_node: Node) -> None:
        assert_joy_topic_switch(
            node=singleton_node,
            expected_topic="/panther/joy",
            button_config=[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        )

    @pytest.mark.parametrize(
        "axes, direction",
        [
            ([0.0, 1.0, 0.0, 0.0, 0.0], "x"),
            ([0.0, 0.0, 0.0, 1.0, 0.0], "z"),
        ],
    )
    def test_move_panther_with_joy(
        self,
        axes: list[float],
        direction: str,
        test_node: Node,
        movement_threshold: float = 0.01,
    ) -> None:
        """Tests the linear movements of the hand while controlling with the joystick.
        Assert if it moves above a certain movement_threshold."""
        pose = PoseStamped()
        pose.header.frame_id = "panther/fr3_hand"
        first_pose = call_express_pose_in_other_frame(
            node=test_node, pose=pose, target_frame="franka/fr3_link0"
        ).pose.pose

        pub = test_node.create_publisher(Joy, "/joy", 10)

        msg = Joy()
        msg.axes = axes
        msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1]
        pub.publish(msg)
        rclpy.spin_once(test_node, timeout_sec=0.1)
        time.sleep(1)  # small delay for the robot to move

        pose = PoseStamped()
        pose.header.frame_id = "franka/fr3_hand"
        moved_pose = call_express_pose_in_other_frame(
            node=test_node, pose=pose, target_frame="franka/fr3_link0"
        ).pose.pose

        delta = getattr(moved_pose.position, direction) - getattr(
            first_pose.position, direction
        )

        assert abs(delta) > movement_threshold, (
            f"{direction} position did not change after input. Δ = {delta:.4f}"
        )
