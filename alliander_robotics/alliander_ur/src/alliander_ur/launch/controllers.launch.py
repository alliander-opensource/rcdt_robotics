# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from alliander_utilities.config_objects import Arm
from alliander_utilities.launch_argument import LaunchArgument
from alliander_utilities.launch_utils import SKIP
from alliander_utilities.register import Register
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

TIMEOUT = 100

platform_arg = LaunchArgument("platform_config", "")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the UR controllers.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    arm_config = Arm.from_str(platform_arg.string_value(context))

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--switch-timeout",
            str(TIMEOUT),
        ],
        namespace=arm_config.namespace,
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--switch-timeout",
            str(TIMEOUT),
        ],
        namespace=arm_config.namespace,
    )

    io_and_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "io_and_status_controller",
            "--switch-timeout",
            str(TIMEOUT),
        ],
        namespace=arm_config.namespace,
    )

    wave_node = Node(
        package="alliander_arm_controllers",
        executable="arm_wave_node",
        parameters=[
            {
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ]
            },
            # based on initial UR joint positions
            {
                "initial_joint_positions": [
                    0.0,
                    -1.57,
                    1.57,
                    -1.57,
                    -1.57,
                    -1.57,
                ]
            },
            {"control_gripper": False},
            # bottom joint (yaw) and middle joint (up and down)
            {"wave_joint_indices": [0, 2]},
            {"wave_amplitudes": [0.4, 0.5]},
            {"wave_period_sec": 8.0},
        ],
        namespace=arm_config.namespace,
    )

    return [
        Register.on_exit(joint_state_broadcaster_spawner, context),
        Register.on_exit(joint_trajectory_controller_spawner, context),
        Register.on_exit(io_and_status_controller_spawner, context)
        if not arm_config.simulation
        else SKIP,
        Register.on_start(wave_node, context)
        if arm_config.movement == "wave"
        else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the UR controllers.

    Returns:
        LaunchDescription: The launch description containing the nodes and actions.
    """
    return LaunchDescription(
        [
            platform_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
