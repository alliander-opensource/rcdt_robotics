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
use_velodyne_arg = LaunchArgument("velodyne", False, [True, False])
use_slam_arg = LaunchArgument("slam", False, [True, False])
use_collision_monitor_arg = LaunchArgument("collision_monitor", False, [True, False])
use_navigation_arg = LaunchArgument("navigation", False, [True, False])
scale_speed_arg = LaunchArgument(
    "scale_speed", default_value=0.4, min_value=0.0, max_value=1.0
)
panther_xyz_arg = LaunchArgument("panther_xyz", "0,0,0.2")
global_map_arg = LaunchArgument(
    "map", "map.yaml", ["map.yaml", "ipkw.yaml", "ipkw_buiten.yaml"]
)
use_ui_arg = LaunchArgument("ui", False, [True, False])


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Panther robot.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)
    load_gazebo_ui = load_gazebo_ui_arg.bool_value(context)
    use_rviz = use_rviz_arg.bool_value(context)
    world = world_arg.string_value(context)
    use_velodyne = use_velodyne_arg.bool_value(context)
    use_slam = use_slam_arg.bool_value(context)
    use_collision_monitor = use_collision_monitor_arg.bool_value(context)
    use_navigation = use_navigation_arg.bool_value(context)
    scale_speed = scale_speed_arg.float_value(context)
    panther_xyz = panther_xyz_arg.string_value(context)
    global_map = global_map_arg.string_value(context)
    use_ui = use_ui_arg.bool_value(context)

    namespace = "panther"
    ns = f"/{namespace}" if namespace else ""

    if use_slam or use_collision_monitor or use_navigation:
        use_velodyne = True

    launch_arguments = {
        "simulation": str(use_sim),
        "load_gazebo_ui": str(load_gazebo_ui),
        "world": world,
        "panther_xyz": panther_xyz,
    }
    if use_velodyne:
        launch_arguments["child"] = "velodyne"

    core = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "core.launch.py"),
        launch_arguments=launch_arguments,
    )

    controllers = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "controllers.launch.py")
    )

    if use_navigation and use_collision_monitor:
        rviz_display_config = "panther_navigation_and_collision_monitor.rviz"
    elif use_collision_monitor:
        rviz_display_config = "panther_collision_monitor.rviz"
    elif use_navigation:
        rviz_display_config = "panther_navigation.rviz"
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
        launch_arguments={
            "simulation": str(use_sim),
            "robots": "panther",
            "scale_speed": str(scale_speed),
            "collision_monitor": str(use_collision_monitor),
        },
    )

    velodyne = RegisteredLaunchDescription(
        get_file_path("rcdt_sensors", ["launch"], "velodyne.launch.py"),
        launch_arguments={"simulation": str(use_sim)},
    )

    slam = RegisteredLaunchDescription(
        get_file_path("slam_toolbox", ["launch"], "online_async_launch.py"),
        launch_arguments={
            "slam_params_file": get_file_path(
                "rcdt_panther", ["config"], "slam_params.yaml"
            ),
        },
    )

    nav2 = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "nav2.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "autostart": str(True),
            "slam": str(use_slam),
            "collision_monitor": str(use_collision_monitor),
            "navigation": str(use_navigation),
            "global_map": str(global_map),
        },
    )

    vizanti_server_launch = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "vizanti.launch.py")
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        Register.group(velodyne, context) if use_velodyne else SKIP,
        Register.group(core, context) if use_sim else SKIP,
        Register.group(controllers, context) if use_sim else SKIP,
        Register.group(joystick, context),
        Register.group(slam, context) if use_slam else SKIP,
        Register.group(nav2, context)
        if use_navigation or use_collision_monitor
        else SKIP,
        Register.group(rviz, context) if use_rviz else SKIP,
        Register.group(vizanti_server_launch, context) if use_ui else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Panther robot.

    Returns:
        LaunchDescription: The launch description for the Panther robot.
    """
    return LaunchDescription(
        [
            scale_speed_arg.declaration,
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            use_rviz_arg.declaration,
            world_arg.declaration,
            use_velodyne_arg.declaration,
            use_slam_arg.declaration,
            use_collision_monitor_arg.declaration,
            use_navigation_arg.declaration,
            panther_xyz_arg.declaration,
            global_map_arg.declaration,
            use_ui_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
