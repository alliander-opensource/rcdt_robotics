# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from rcdt_utilities.launch_utils import LaunchArgument, get_file_path

simulation_arg = LaunchArgument("simulation", True, [True, False])
arm_controller_arg = LaunchArgument("arm_controller", "fr3_arm_controller")
gripper_controller_arg = LaunchArgument("gripper_controller", "fr3_gripper")

controllers_config = get_file_path(
    "rcdt_franka", ["config"], "simulation_controllers.yaml"
)
gripper_config = get_file_path("franka_gripper", ["config"], "franka_gripper_node.yaml")


def launch_setup(context: LaunchContext) -> None:
    simulation = simulation_arg.value(context)
    arm_controller = arm_controller_arg.value(context)
    gripper_controller = gripper_controller_arg.value(context)

    controllers_yaml = (
        "simulation_controllers.yaml" if simulation else "robot_controllers.yaml"
    )
    controllers_config = get_file_path("rcdt_franka", ["config"], controllers_yaml)

    fr3_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller", "-p", controllers_config],
    )

    if simulation:
        fr3_gripper = Node(
            package="rcdt_franka",
            executable="franka_gripper_simulation_node.py",
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

    gripper_action = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "-p", controllers_config],
    )

    skip = LaunchDescriptionEntity()
    return [
        fr3_arm_controller if arm_controller == "fr3_arm_controller" else skip,
        fr3_gripper if gripper_controller == "fr3_gripper" else skip,
        gripper_action if gripper_controller == "fr3_gripper" and simulation else skip,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            arm_controller_arg.declaration,
            gripper_controller_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
