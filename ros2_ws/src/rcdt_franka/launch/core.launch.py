# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import (
    LaunchArgument,
    get_file_path,
    get_robot_description,
)

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "empty_camera.sdf")
use_realsense_arg = LaunchArgument("realsense", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    world = str(world_arg.value(context))
    use_realsense = use_realsense_arg.value(context)

    xacro_path = get_file_path("rcdt_franka", ["urdf"], "franka.urdf.xacro")
    xacro_arguments = {}
    xacro_arguments["simulation"] = "true" if use_sim else "false"
    xacro_arguments["load_realsense"] = "true"
    robot_description = get_robot_description(xacro_path, xacro_arguments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    if use_sim:
        robot = IncludeLaunchDescription(
            get_file_path("rcdt_gazebo", ["launch"], "gazebo_robot.launch.py"),
            launch_arguments={
                "realsense": str(use_realsense),
                "load_gazebo_ui": str(load_gazebo_ui),
                "world": world,
            }.items(),
        )
    else:
        robot = IncludeLaunchDescription(
            get_file_path("rcdt_franka", ["launch"], "robot.launch.py")
        )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world",
        arguments=["--frame-id", "world", "--child-frame-id", "base"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    controllers = IncludeLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "controllers.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "arm_controller": "fr3_arm_controller",
            "gripper_controller": "fr3_gripper",
        }.items(),
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        robot_state_publisher,
        robot,
        static_transform_publisher,
        joint_state_broadcaster,
        controllers,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            world_arg.declaration,
            use_realsense_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
