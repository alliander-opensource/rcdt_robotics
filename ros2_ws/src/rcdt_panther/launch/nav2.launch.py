# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path
from rcdt_utilities.register import Register

use_sim_arg = LaunchArgument("simulation", True, [True, False])
autostart_arg = LaunchArgument("autostart", True, [True, False])
use_respawn_arg = LaunchArgument("use_respawn", False, [True, False])
use_slam_arg = LaunchArgument("slam", False, [True, False])
use_collision_monitor_arg = LaunchArgument("collision_monitor", False, [True, False])
use_navigation_arg = LaunchArgument("navigation", False, [True, False])
controller_arg = LaunchArgument(
    "controller",
    "vector_pursuit",
    [
        "dwb",
        "graceful_motion",
        "mppi",
        "pure_pursuit",
        "rotation_shim",
        "vector_pursuit",
    ],
)


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the navigation stack.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)
    autostart = autostart_arg.bool_value(context)
    use_respawn = use_respawn_arg.bool_value(context)
    use_slam = use_slam_arg.bool_value(context)
    use_collision_monitor = use_collision_monitor_arg.bool_value(context)
    use_navigation = use_navigation_arg.bool_value(context)
    controller = controller_arg.string_value(context)

    lifecycle_nodes = []

    if use_collision_monitor:
        lifecycle_nodes.append("collision_monitor")

    if not use_slam:
        lifecycle_nodes.extend(
            [
                "map_server",
                "amcl",
            ]
        )

    if use_navigation:
        lifecycle_nodes.extend(
            [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
            ]
        )

    amcl_params = RewrittenYaml(
        source_file=get_file_path("rcdt_panther", ["config", "nav2"], "amcl.yaml"),
        param_rewrites={},
    )

    local_costmap_params = RewrittenYaml(
        source_file=get_file_path(
            "rcdt_panther", ["config", "nav2"], "local_costmap.yaml"
        ),
        param_rewrites={},
    )

    global_costmap_params = RewrittenYaml(
        source_file=get_file_path(
            "rcdt_panther", ["config", "nav2"], "global_costmap.yaml"
        ),
        param_rewrites={},
    )

    controller_server_params = RewrittenYaml(
        source_file=get_file_path(
            "rcdt_panther", ["config", "nav2"], "controller_server.yaml"
        ),
        param_rewrites={},
    )

    planner_server_params = RewrittenYaml(
        source_file=get_file_path(
            "rcdt_panther", ["config", "nav2"], "planner_server.yaml"
        ),
        param_rewrites={},
    )

    behavior_server_params = RewrittenYaml(
        source_file=get_file_path(
            "rcdt_panther", ["config", "nav2"], "behavior_server.yaml"
        ),
        param_rewrites={},
    )

    follow_path_params = load_follow_path_parameters(controller)

    bt_navigator_params = RewrittenYaml(
        source_file=get_file_path(
            "rcdt_panther", ["config", "nav2"], "bt_navigator.yaml"
        ),
        param_rewrites={
            "default_nav_to_pose_bt_xml": get_file_path(
                "rcdt_panther", ["config", "nav2"], "behavior_tree.xml"
            )
        },
    )

    map_filename = "map.yaml" if use_sim else "ipkw.yaml"
    map_yaml = get_file_path("rcdt_panther", ["config", "maps"], map_filename)

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[{"yaml_filename": map_yaml}],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        parameters=[amcl_params],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[local_costmap_params, controller_server_params, follow_path_params],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[global_costmap_params, planner_server_params],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[behavior_server_params],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[bt_navigator_params],
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
    )

    collision_monitor_params = RewrittenYaml(
        source_file=get_file_path(
            "rcdt_panther", ["config", "nav2"], "collision_monitor.yaml"
        ),
        param_rewrites={},
    )

    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[collision_monitor_params],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{"autostart": autostart}, {"node_names": lifecycle_nodes}],
    )

    waypoint_follower_controller = Node(
        package="rcdt_panther", executable="waypoint_follower_controller.py"
    )

    pub_topic = (
        "/panther/cmd_vel" if not use_collision_monitor else "/panther/cmd_vel_raw"
    )

    return [
        SetRemap(src="/cmd_vel", dst=pub_topic),
        Register.on_start(map_server, context) if not use_slam else SKIP,
        Register.on_start(amcl, context) if not use_slam else SKIP,
        Register.on_start(controller_server, context) if use_navigation else SKIP,
        Register.on_start(planner_server, context) if use_navigation else SKIP,
        Register.on_start(behavior_server, context) if use_navigation else SKIP,
        Register.on_start(bt_navigator, context) if use_navigation else SKIP,
        Register.on_start(waypoint_follower, context) if use_navigation else SKIP,
        Register.on_start(collision_monitor_node, context)
        if use_collision_monitor
        else SKIP,
        Register.on_log(lifecycle_manager, "Managed nodes are active", context),
        Register.on_log(waypoint_follower_controller, "Controller is ready.", context),
    ]


def load_follow_path_parameters(plugin: str = "dwb") -> RewrittenYaml:
    """Load the follow path parameters for the specified plugin.

    Supported plugins are listed here: https://docs.nav2.org/plugins/index.html#controllers
    But only the types declared by nav2 (without extra installs) are supported:
        - dwb_core::DWBLocalPlanner
        - nav2_graceful_controller::GracefulController
        - nav2_mppi_controller::MPPIController
        - nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
        - nav2_rotation_shim_controller::RotationShimController

    Args:
        plugin (str): The name of the plugin for which to load parameters.

    Returns:
        RewrittenYaml: The rewritten YAML configuration for the specified plugin.
    """
    return RewrittenYaml(
        source_file=get_file_path(
            "rcdt_panther", ["config", "nav2", "controllers"], f"{plugin}.yaml"
        ),
        param_rewrites={},
    )


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the navigation stack.

    Returns:
        LaunchDescription: The launch description containing the actions to be executed.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            use_collision_monitor_arg.declaration,
            autostart_arg.declaration,
            use_respawn_arg.declaration,
            controller_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
