# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path
import os 
from launch import LaunchContext, LaunchDescription
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from launch.actions import OpaqueFunction


def launch_setup(context: LaunchContext):
    namespace = "franka"
    ns = f"/{namespace}" if namespace else ""


    hostname = os.getenv('FRANKA_HOSTNAME', '')
    username = os.getenv('FRANKA_USERNAME', 'admin')
    password = os.getenv('FRANKA_PASSWORD', '')

    if not hostname or not username or not password:
        raise RuntimeError(
            "You must set FRANKA_HOSTNAME and FRANKA_PASSWORD in your environment."
        )

    # only include the node if we actually have a hostname and password
    franka_lock_unlock = Node(
        name="franka_lock_unlock",
        package="franka_lock_unlock",
        executable="franka_lock_unlock.py",
        output='screen',
        arguments=[
            hostname,
            username,
            password,
            '-u','-l', '-w','-r',  '-p','-c'            
        ],
        respawn=True
    )

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
    
    return[
        Register.on_log(franka_lock_unlock,"Keeping persistent connection...", context),
        Register.on_start(settings_setter, context),
        Register.on_start(ros2_control_node, context),
        Register.on_start(joint_state_publisher, context) 
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
