# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from launch_testing_ros.wait_for_topics import WaitForTopics
from rcdt_messages.srv import MoveToConfiguration
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

def call_trigger_service(node: rclpy.node.Node, service_name: str) -> bool:
    """Call a trigger service and return True if the service was called successfully."""
    client = node.create_client(Trigger, service_name)
    if not client.wait_for_service():
        raise RuntimeError(f"Service {service_name} not available")

    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future=future)
    return future.result() is not None

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
    topic_list = [("panther/joint_states", JointState)]
    wait_for_topics = WaitForTopics(topic_list, timeout=10.0)
    assert wait_for_topics.wait()
    msg = wait_for_topics.received_messages("panther/joint_states")[0]
    idx = msg.name.index(name)
    position = msg.position[idx]
    wait_for_topics.shutdown()
    return position