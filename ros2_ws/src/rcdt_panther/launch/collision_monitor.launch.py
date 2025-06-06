# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path
from rcdt_utilities.register import Register

namespace_arg = LaunchArgument("namespace", "", None)
use_sim_time_arg = LaunchArgument("use_sim_time", True, [True, False])
autostart_arg = LaunchArgument("autostart", True, [True, False])
print(get_file_path("rcdt_panther", ["config"], "collision_monitor.yaml"))
params_file_arg = LaunchArgument(
    "params_file", get_file_path("rcdt_panther", ["config"], "collision_monitor.yaml")
)
use_respawn_arg = LaunchArgument("use_respawn", False, [True, False])
log_level_arg = LaunchArgument("log_level", "info")

def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Collision Monitor.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    namespace = namespace_arg.string_value(context)
    use_sim_time = use_sim_time_arg.bool_value(context)
    autostart = autostart_arg.bool_value(context)
    params_file = params_file_arg.string_value(context)
    log_level = log_level_arg.string_value(context)

    lifecycle_nodes = ["collision_monitor"]

    param_substitutions = {
        "use_sim_time": str(use_sim_time)
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

    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        output="screen",
        emulate_tty=True,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    return [
        Register.on_start(collision_monitor_node, context),
        Register.on_log(lifecycle_manager_node, "Creating bond timer", context),
    ]

def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Collision Monitor.

    Returns:
        LaunchDescription: The launch description containing the actions to be executed.
    """
    return LaunchDescription([
        namespace_arg.declaration,
        use_sim_time_arg.declaration,
        params_file_arg.declaration,
        autostart_arg.declaration,
        use_respawn_arg.declaration,
        log_level_arg.declaration,
        OpaqueFunction(function=launch_setup),
    ])
