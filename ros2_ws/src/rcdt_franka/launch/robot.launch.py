# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, Shutdown
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import Register

namespace = "franka"
ns = f"/{namespace}" if namespace else ""

franka_controllers = param_file = get_file_path(
    "rcdt_franka", ["config"], "controllers.yaml"
)


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Franka robot.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
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

    return [
        settings_setter,
        Register.on_start(ros2_control_node, context),
        Register.on_start(joint_state_publisher, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Franka robot controllers.

    Returns:
        LaunchDescription: The launch description containing the Franka controllers.
    """
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
