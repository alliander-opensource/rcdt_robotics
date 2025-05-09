# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import (
    SKIP,
    LaunchArgument,
    get_file_path,
    get_robot_description,
)

use_sim_arg = LaunchArgument("simulation", True, [True, False])
start_robot_arg = LaunchArgument("start_robot", True, [True, False])
base_link_arg = LaunchArgument("base_link", "world")
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "empty_camera.sdf")
use_realsense_arg = LaunchArgument("realsense", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    start_robot = start_robot_arg.value(context)
    base_link = base_link_arg.value(context)
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    world = str(world_arg.value(context))
    use_realsense = use_realsense_arg.value(context)

    namespace = "franka"
    frame_prefix = namespace + "/" if namespace else ""

    xacro_path = get_file_path("rcdt_franka", ["urdf"], "franka.urdf.xacro")
    xacro_arguments = {}
    xacro_arguments["simulation"] = "true" if use_sim else "false"
    xacro_arguments["namespace"] = namespace
    xacro_arguments["parent"] = base_link
    xacro_arguments["load_realsense"] = "true" if use_realsense else "false"
    robot_description = get_robot_description(xacro_path, xacro_arguments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[
            robot_description,
            {"frame_prefix": frame_prefix},
        ],
    )

    if use_sim:
        robot = IncludeLaunchDescription(
            get_file_path("rcdt_gazebo", ["launch"], "gazebo_robot.launch.py"),
            launch_arguments={
                "world": world,
                "robots": namespace,
                "realsense": str(use_realsense),
                "load_gazebo_ui": str(load_gazebo_ui),
            }.items(),
        )
    else:
        robot = IncludeLaunchDescription(
            get_file_path("rcdt_franka", ["launch"], "robot.launch.py")
        )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        namespace=namespace,
    )

    # Create a tf frame called 'base', required for the MotionPlanning plugin in Rviz:
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world",
        arguments=[
            "--frame-id",
            frame_prefix + "fr3_link0",
            "--child-frame-id",
            "base",
        ],
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        robot_state_publisher,
        robot if start_robot else SKIP,
        joint_state_broadcaster,
        static_transform_publisher if start_robot else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            start_robot_arg.declaration,
            base_link_arg.declaration,
            load_gazebo_ui_arg.declaration,
            world_arg.declaration,
            use_realsense_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
