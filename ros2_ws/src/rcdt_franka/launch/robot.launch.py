# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path

namespace = "franka"
ns = f"/{namespace}" if namespace else ""

franka_controllers = param_file = get_file_path(
    "rcdt_franka", ["config"], "controllers.yaml"
)

ros2_control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        franka_controllers,
        {"arm_id": "fr3"},
    ],
    remappings=[
        (f"{ns}/controller_manager/robot_description", f"{ns}/robot_description"),
        (f"{ns}/joint_states", f"{ns}/fr3_arm/joint_states"),
    ],
    namespace=namespace,
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
            "source_list": [
                f"{ns}/fr3_arm/joint_states",
                f"{ns}/fr3_gripper/joint_states",
            ],
            "rate": 30,
        }
    ],
    namespace=namespace,
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            settings_setter,
            ros2_control_node,
            joint_state_publisher,
        ]
    )
