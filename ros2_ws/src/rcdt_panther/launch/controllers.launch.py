# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.register import Register


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Panther controllers.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    namespace = "panther"

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        namespace=namespace,
    )

    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "drive_controller",
            "--controller-manager",
            "controller_manager",
            "--controller-ros-args",
            "--remap drive_controller/cmd_vel:=cmd_vel",
            "--controller-ros-args",
            "--remap drive_controller/odom:=odometry/wheels",
            "--controller-ros-args",
            "--remap drive_controller/transition_event:=drive_controller/_transition_event",
            "--controller-ros-args",
            "--remap imu_broadcaster/imu:=imu/data",
            "--controller-ros-args",
            "--remap imu_broadcaster/transition_event:=imu_broadcaster/_transition_event",
            "--controller-ros-args",
            "--remap joint_state_broadcaster/transition_event:=joint_state_broadcaster/_transition_event",
        ],
        namespace=namespace,
    )

    return [
        Register.on_exit(joint_state_broadcaster_spawner, context),
        Register.on_exit(drive_controller_spawner, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Panther controllers.

    Returns:
        LaunchDescription: The launch description containing the Panther controllers.
    """
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
