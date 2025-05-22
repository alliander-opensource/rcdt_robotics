# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcdt_messages.srv._move_to_configuration import MoveToConfiguration
from rcdt_utilities.test_utils import (
    create_ready_action_client,
    create_ready_service_client,
)
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future
from trajectory_msgs.msg import JointTrajectoryPoint


def call_move_to_configuration_service(node: Node, configuration: str, timeout:int) -> bool:
    """Call the move_to_configuration service and return True if a response from the service was received."""
    client = create_ready_service_client(
        node, MoveToConfiguration, "franka/moveit_manager/move_to_configuration", timeout_sec=timeout
    )
    request = MoveToConfiguration.Request()
    request.configuration = configuration
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future=future)
    return future.result() is not None


def follow_joint_trajectory_goal(
    node: Node,
    positions: list[float],
    controller: str,
    timeout:int,
    time_from_start: int = 3,
) -> None:
    """Test sending a joint trajectory goal to the arm controller."""

    action_client = create_ready_action_client(
        node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory", timeout = timeout
    )

    goal_msg = FollowJointTrajectory.Goal()
    goal_msg.trajectory.joint_names = [
        "fr3_joint1",
        "fr3_joint2",
        "fr3_joint3",
        "fr3_joint4",
        "fr3_joint5",
        "fr3_joint6",
        "fr3_joint7",
    ]

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = Duration(sec=time_from_start, nanosec=0)

    goal_msg.trajectory.points.append(point)

    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    goal_handle: ClientGoalHandle = future.result()
    assert goal_handle.accepted

    result_future: Future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout)
