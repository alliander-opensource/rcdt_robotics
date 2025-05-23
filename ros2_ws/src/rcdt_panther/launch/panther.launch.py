# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import SetParameter
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
world_arg = LaunchArgument("world", "walls.sdf")
use_collision_monitor_arg = LaunchArgument("collision_monitor", False, [True, False])
use_velodyne_arg = LaunchArgument("velodyne", False, [True, False])
use_slam_arg = LaunchArgument("slam", False, [True, False])
use_nav2_arg = LaunchArgument("nav2", False, [True, False])


def launch_setup(context: LaunchContext) -> list:
    use_sim = use_sim_arg.value(context)
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    use_rviz = use_rviz_arg.value(context)
    world = str(world_arg.value(context))
    use_collision_monitor = use_collision_monitor_arg.value(context)
    use_velodyne = use_velodyne_arg.value(context)
    use_slam = use_slam_arg.value(context)
    use_nav2 = use_nav2_arg.value(context)

    namespace = "panther"
    ns = f"/{namespace}" if namespace else ""

    if use_collision_monitor:
        use_velodyne = True

    if use_slam:
        use_velodyne = True

    if use_nav2:
        use_velodyne = True
        use_slam = True

    core = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "core.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "load_gazebo_ui": str(load_gazebo_ui),
            "velodyne": str(use_velodyne),
            "world": world,
        },
    )

    controllers = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "controllers.launch.py")
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
    rviz = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={
            "rviz_frame": "map" if use_slam else f"{ns}/odom",
            "rviz_display_config": rviz_display_config,
        },
    )

    joystick = RegisteredLaunchDescription(
        get_file_path("rcdt_joystick", ["launch"], "joystick.launch.py"),
        launch_arguments={"robots": "panther"},
    )

    slam = RegisteredLaunchDescription(
        get_file_path("slam_toolbox", ["launch"], "online_async_launch.py"),
        launch_arguments={
            "slam_params_file": get_file_path(
                "rcdt_panther", ["config"], "slam_params.yaml"
            ),
        },
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        Register.group(core, context) if use_sim else SKIP,
        Register.group(controllers, context) if use_sim else SKIP,
        Register.group(joystick, context),
        Register.group(rviz, context) if use_rviz else SKIP,
        Register.group(slam, context) if use_slam else SKIP,
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
