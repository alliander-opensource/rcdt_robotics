# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path

use_sim_arg = LaunchArgument("simulation", True, [True, False])


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the collision monitor.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)

    collision_monitor = IncludeLaunchDescription(
        get_file_path(
            "nav2_collision_monitor", ["launch"], "collision_monitor_node.launch.py"
        ),
        launch_arguments={
            "use_sim_time": str(use_sim),
            "params_file": "/home/rcdt/rcdt_robotics/ros2_ws/src/rcdt_panther/config/collision_monitor.yaml",
        }.items(),
    )

    return [collision_monitor]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the collision monitor.

    Returns:
        LaunchDescription: The launch description for the collision monitor.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
