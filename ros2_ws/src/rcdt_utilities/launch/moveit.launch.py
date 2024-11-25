# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import (
    LaunchArgument,
    get_yaml,
    get_file_path,
    get_moveit_parameters,
)

SKIP = LaunchDescriptionEntity()

use_sim_arg = LaunchArgument("simulation", True, [True, False])
moveit_mode_arg = LaunchArgument("moveit", "off", ["node", "rviz", "servo", "off"])
moveit_package_name_arg = LaunchArgument("moveit_package_name", "")
servo_params_package_arg = LaunchArgument("servo_params_package", "rcdt_franka")


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    moveit_mode = moveit_mode_arg.value(context)
    package_name = moveit_package_name_arg.value(context)
    servo_params_package = servo_params_package_arg.value(context)

    moveit_config = get_moveit_parameters(
        robot_name="fr3",
        package_name=package_name,
        mode=moveit_mode,
    )

    # Moveit as node:
    moveit_node = Node(
        package="rcdt_utilities",
        executable="moveit_controller_node.py",
        parameters=[moveit_config, {"use_sim_time": use_sim}],
    )

    # Moveit in rviz:
    moveit_rviz = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config],
    )

    # Moveit servo:
    file = get_file_path(servo_params_package, ["config"], "servo_params.yaml")
    servo_config = get_yaml(file)
    servo_params = {"moveit_servo": servo_config}
    moveit_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            moveit_config,
        ],
    )

    set_servo_command_type = ExecuteProcess(
        cmd=[
            [
                "ros2 service call ",
                "/servo_node/switch_command_type ",
                "moveit_msgs/srv/ServoCommandType ",
                "'{command_type: 1}'",
            ]
        ],
        shell=True,
    )

    # Select moveit:
    moveit_selector = {
        "node": moveit_node,
        "rviz": moveit_rviz,
        "servo": moveit_servo,
        "off": SKIP,
    }
    moveit = moveit_selector[moveit_mode]

    return [
        moveit,
        set_servo_command_type if moveit_mode == "servo" else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            moveit_mode_arg.declaration,
            moveit_package_name_arg.declaration,
            servo_params_package_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
