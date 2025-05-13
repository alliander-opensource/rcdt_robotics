# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from launch_testing_ros.wait_for_topics import WaitForTopics
from rcdt_messages.srv import MoveToConfiguration
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


def get_joint_position(name: str) -> float:
    """Get the joint position of a joint by name. This is done by calling the
    /joint_states topic and parsing the output. This is a workaround for the fact
    that the joint states are not published in a format that can be easily parsed.
    Also --field does not work with the /joint_states topic.

    Args:
        name (str): The name of the joint.
    Returns:
        float: The position of the joint.
    """
    topic_list = [("joint_states", JointState)]
    wait_for_topics = WaitForTopics(topic_list, timeout=10.0)
    assert wait_for_topics.wait()
    msg = wait_for_topics.received_messages("joint_states")[0]
    idx = msg.name.index(name)
    position = msg.position[idx]
    wait_for_topics.shutdown()
    return position


def call_trigger_service(node: Node, service_name: str) -> bool:
    """Call a trigger service and return True if the service was called successfully."""
    client = node.create_client(Trigger, service_name)
    if not client.wait_for_service():
        raise RuntimeError(f"Service {service_name} not available")

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future=future)
    return future.result() is not None


def call_move_gripper_service(
    node: Node, width: float, action_name: str
) -> bool:
    """Call the gripper to go to a defined width."""
    action_client = ActionClient(node, GripperCommand, action_name=action_name)

    if not action_client.wait_for_server():
        raise RuntimeError(f"Action server {action_name} not available")

    goal_msg = GripperCommand.Goal()
    goal_msg.command.position = width

    future = action_client.send_goal_async(goal_msg)

    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()
    if not goal_handle.accepted:
        return False

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    return result.reached_goal


def follow_joint_trajectory_goal(
    singleton_node: Node,
    positions: list[float],
    controller: str = "fr3_arm_controller",
    time_from_start: int = 3,
) -> bool:
    """Test sending a joint trajectory goal to the arm controller."""

    action_client = ActionClient(
        singleton_node,
        FollowJointTrajectory,
        f"/{controller}/follow_joint_trajectory",
    )

    if not action_client.wait_for_server():
        raise RuntimeError("Action server not available")

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
    rclpy.spin_until_future_complete(singleton_node, future)
    goal_handle = future.result()
    assert goal_handle.accepted

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(singleton_node, result_future)
    result = result_future.result().result
    return result
