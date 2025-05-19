# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


from rcdt_utilities.launch_utils import assert_for_message
from rcdt_utilities.test_utils import call_trigger_service, assert_joy_topic_switch
from rclpy.node import Node
from sensor_msgs.msg import JointState
from utils import EndToEndUtils


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
