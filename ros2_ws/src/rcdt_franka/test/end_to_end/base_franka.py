# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rcdt_utilities.launch_utils import assert_for_message, assert_for_node
from rcdt_utilities.test_utils import wait_for_register
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from utils import EndToEndUtils, call_express_pose_in_other_frame


class FrankaFullTests(EndToEndUtils):
    def test_wait_for_register(self, pytestconfig: pytest.Config) -> None:
        wait_for_register(pytestconfig)

    def test_joint_states_published(self) -> None:
        """Test that joint states are published. This is a basic test to check that the
        launch file is working and that the robot is publishing joint states."""
        assert_for_message(JointState, "franka/joint_states", 60)

    def test_ready_to_start(self, test_node: Node) -> None:
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
            self.wait_until_active(
                node=test_node, controller_name="gripper_action_controller"
            )
            is True
        )

    def test_switch_joy_to_franka_topic(self, test_node: Node) -> None:
        """Test to see if the switch to Franka mode is correct."""
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        result = {}

        def listener_callback(msg: String) -> None:
            result["state"] = msg.data

        # Subscribe to the correct topic
        test_node.create_subscription(
            String,
            "/joy_topic_manager/state",  # this matches ~/state in JoyTopicManager
            listener_callback,
            qos_profile=latched_qos,
        )

        pub = test_node.create_publisher(Joy, "/joy", 10)
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        pub.publish(msg)
        timeout_sec = 10.0
        start_time = time.time()
        while (
            result.get("state") != "/franka/joy"
            and time.time() - start_time < timeout_sec
        ):
            rclpy.spin_once(test_node, timeout_sec=0.1)
            time.sleep(0.1)

        assert result.get("state") == "/franka/joy", (
            f"Unexpected state: {result.get('state')}"
        )

    @pytest.mark.parametrize(
        "buttons, expected_value",
        [
            ([1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], 0.04),
            ([1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1], 0.0),
        ],
    )
    def test_joy_gripper_node(
        self,
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

        reached_goal, joint_value = self.wait_until_reached_joint(
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
        self,
        axes: list[float],
        direction: str,
        test_node: Node,
        movement_threshold: float = 0.01,
    ) -> None:
        """Tests the linear movements of the hand while controlling with the joystick.
        Assert if it moves above a certain movement_threshold."""
        pose = PoseStamped()
        pose.header.frame_id = "franka/fr3_hand"
        first_pose = call_express_pose_in_other_frame(
            node=test_node, pose=pose, target_frame="franka/fr3_link0"
        ).pose.pose

        pub = test_node.create_publisher(Joy, "/joy", 10)

        msg = Joy()
        msg.axes = axes
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
