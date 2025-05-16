# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


from launch_testing_ros.wait_for_topics import WaitForTopics
from sensor_msgs.msg import JointState


def get_joint_position(name_space: str, joint: str) -> float:
    """Get the joint position of a joint by name. This is done by calling the
    /joint_states topic and parsing the output. This is a workaround for the fact
    that the joint states are not published in a format that can be easily parsed.
    Also --field does not work with the /joint_states topic.

    Args:
        name_space (str); The name space of the robot.
        joint (str): The name of the joint.
    Returns:
        float: The position of the joint.
    """
    topic_list = [(f"{name_space}/joint_states", JointState)]
    wait_for_topics = WaitForTopics(topic_list, timeout=10.0)
    assert wait_for_topics.wait()
    msg: JointState = wait_for_topics.received_messages(f"{name_space}/joint_states")[0]
    idx = msg.name.index(joint)
    position = msg.position[idx]
    wait_for_topics.shutdown()
    return position
