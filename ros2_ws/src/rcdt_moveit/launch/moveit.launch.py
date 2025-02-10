# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path, get_yaml

robot_name_arg = LaunchArgument("robot_name", "")
moveit_package_name_arg = LaunchArgument("moveit_package_name", "")
servo_params_package_arg = LaunchArgument("servo_params_package", "rcdt_franka")


def launch_setup(context: LaunchContext) -> None:
    robot_name = robot_name_arg.value(context)
    package_name = moveit_package_name_arg.value(context)
    servo_params_package = servo_params_package_arg.value(context)

    moveit_config = MoveItConfigsBuilder(robot_name, package_name=package_name)
    moveit_config.trajectory_execution(
        get_file_path(package_name, ["config"], "moveit_controllers.yaml")
    )
    moveit_config.moveit_cpp(
        get_file_path(package_name, ["config"], "planning_pipeline.yaml")
    )
    moveit_config = moveit_config.to_dict()

    file = get_file_path(servo_params_package, ["config"], "servo_params.yaml")
    servo_config = get_yaml(file)
    servo_params = {"moveit_servo": servo_config}

    moveit_manager = Node(
        package="rcdt_moveit",
        executable="moveit_manager",
        output="screen",
        parameters=[moveit_config],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config],
    )

    moveit_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[moveit_config, servo_params],
    )

    return [move_group, moveit_servo, moveit_manager]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            robot_name_arg.declaration,
            moveit_package_name_arg.declaration,
            servo_params_package_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
