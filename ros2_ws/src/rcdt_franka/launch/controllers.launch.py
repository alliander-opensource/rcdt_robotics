# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path

simulation_arg = LaunchArgument("simulation", True, [True, False])

gripper_config = get_file_path("franka_gripper", ["config"], "franka_gripper_node.yaml")


def launch_setup(context: LaunchContext) -> None:
    simulation = simulation_arg.value(context)

    namespace = "franka"

    fr3_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller"],
        namespace=namespace,
    )

    if simulation:
        fr3_gripper = Node(
            package="rcdt_franka",
            executable="franka_gripper_simulation.py",
            output="screen",
        )
    else:
        fr3_gripper = Node(
            package="franka_gripper",
            executable="franka_gripper_node",
            name="fr3_gripper",
            parameters=[
                {
                    "robot_ip": "172.16.0.2",
                    "joint_names": ["fr3_finger_joint1", "fr3_finger_joint2"],
                },
                gripper_config,
            ],
        )

    gripper_action_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller"],
        namespace=namespace,
    )

    return [
        fr3_arm_controller_spawner,
        fr3_gripper,
        gripper_action_controller_spawner if simulation else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            simulation_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
