# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
world_arg = LaunchArgument("world", "walls.sdf")
use_collision_monitor_arg = LaunchArgument("collision_monitor", False, [True, False])
use_velodyne_arg = LaunchArgument("velodyne", False, [True, False])
use_slam_arg = LaunchArgument("slam", False, [True, False])
use_nav2_arg = LaunchArgument("nav2", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    use_rviz = use_rviz_arg.value(context)
    world = str(world_arg.value(context))
    use_collision_monitor = use_collision_monitor_arg.value(context)
    use_velodyne = use_velodyne_arg.value(context)
    use_slam = use_slam_arg.value(context)
    use_nav2 = use_nav2_arg.value(context)

    if use_collision_monitor:
        use_velodyne = True

    if use_slam:
        use_velodyne = True

    if use_nav2:
        use_velodyne = True
        use_slam = True

    core = IncludeLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "core.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "load_gazebo_ui": str(load_gazebo_ui),
            "velodyne": str(use_velodyne),
            "world": world,
        }.items(),
    )

    if use_nav2:
        rviz_display_config = "panther_nav2.rviz"
    elif use_collision_monitor:
        rviz_display_config = "panther_collision_monitor.rviz"
    elif use_slam:
        rviz_display_config = "panther_slam.rviz"
    elif use_velodyne:
        rviz_display_config = "panther_velodyne.rviz"
    else:
        rviz_display_config = "panther_general.rviz"
    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={
            "rviz_frame": "map" if use_slam else "/panther/odom",
            "rviz_display_config": rviz_display_config,
        }.items(),
    )

    joy = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[
            {"sticky_buttons": True},
        ],
    )

    joy_topic_manager = Node(
        package="rcdt_mobile_manipulator",
        executable="joy_topic_manager.py",
    )

    joy_to_twist_panther = Node(
        package="rcdt_utilities",
        executable="joy_to_twist.py",
        parameters=[
            {"sub_topic": "/panther/joy"},
            {"pub_topic": "/cmd_vel" if use_collision_monitor else "/panther/cmd_vel"},
            {"stamped": False},
            {"config_pkg": "rcdt_panther"},
        ],
    )

    slam = IncludeLaunchDescription(
        get_file_path("slam_toolbox", ["launch"], "online_async_launch.py"),
        launch_arguments={
            "slam_params_file": get_file_path(
                "rcdt_panther", ["config"], "slam_params.yaml"
            ),
        }.items(),
    )

    launch_description = LaunchDescription(
        [
            joy,
            joy_topic_manager,
            joy_to_twist_panther,
            rviz if use_rviz else SKIP,
            slam if use_slam else SKIP,
        ]
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        core if use_sim else SKIP,
        launch_description,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            use_rviz_arg.declaration,
            world_arg.declaration,
            use_collision_monitor_arg.declaration,
            use_velodyne_arg.declaration,
            use_slam_arg.declaration,
            use_nav2_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
