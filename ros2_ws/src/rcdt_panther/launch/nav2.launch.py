# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path
from rcdt_utilities.register import Register

namespace_arg = LaunchArgument("namespace", "", None)
use_sim_time_arg = LaunchArgument("use_sim_time", True, [True, False])
autostart_arg = LaunchArgument("autostart", True, [True, False])
params_file_arg = LaunchArgument(
    "params_file", get_file_path("nav2_bringup", ["params"], "nav2_params.yaml")
)
use_respawn_arg = LaunchArgument("use_respawn", False, [True, False])
log_level_arg = LaunchArgument("log_level", "info")
use_collision_monitor_arg = LaunchArgument("collision_monitor", False, [True, False])


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the navigation stack.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    namespace = namespace_arg.string_value(context)
    use_sim_time = use_sim_time_arg.bool_value(context)
    autostart = autostart_arg.bool_value(context)
    params_file = params_file_arg.string_value(context)
    use_respawn = use_respawn_arg.bool_value(context)
    log_level = log_level_arg.string_value(context)
    use_collision_monitor = use_collision_monitor_arg.bool_value(context)

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    param_substitutions = {
        "use_sim_time": str(use_sim_time),
        "autostart": str(autostart),
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    controller_node = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
    )

    smoother_node = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    planner_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    behavior_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    waypoint_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings
        + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
    )

    lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    pub_topic = (
        "/panther/cmd_vel" if not use_collision_monitor else "/panther/cmd_vel_raw"
    )

    return [
        SetParameter(name="enable_stamped_cmd_vel", value=True),
        SetRemap(src="/cmd_vel", dst=pub_topic),
        Register.on_start(controller_node, context),
        Register.on_start(smoother_node, context),
        Register.on_start(planner_node, context),
        Register.on_start(behavior_node, context),
        Register.on_start(bt_navigator_node, context),
        Register.on_start(waypoint_node, context),
        Register.on_start(velocity_smoother_node, context),
        Register.on_log(lifecycle_node, "Managed nodes are active", context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the navigation stack.

    Returns:
        LaunchDescription: The launch description containing the actions to be executed.
    """
    return LaunchDescription(
        [
            namespace_arg.declaration,
            use_sim_time_arg.declaration,
            params_file_arg.declaration,
            use_collision_monitor_arg.declaration,
            autostart_arg.declaration,
            use_respawn_arg.declaration,
            log_level_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
