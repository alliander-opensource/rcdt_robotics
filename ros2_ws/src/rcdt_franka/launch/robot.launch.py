# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path

franka_controllers = param_file = get_file_path(
    "franka_bringup", ["config"], "controllers.yaml"
)

ros2_control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        franka_controllers,
        {"arm_id": "fr3"},
    ],
    remappings=[
        ("/controller_manager/robot_description", "/robot_description"),
        ("joint_states", "franka/joint_states"),
    ],
    on_exit=Shutdown(),
)

settings_setter = Node(
    package="rcdt_franka",
    executable="settings_setter.py",
)

joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    name="joint_state_publisher",
    parameters=[
        {
            "source_list": ["/franka/joint_states", "fr3_gripper/joint_states"],
            "rate": 30,
        }
    ],
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            settings_setter,
            ros2_control_node,
            joint_state_publisher,
        ]
    )
