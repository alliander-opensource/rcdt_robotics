# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import SetRemap
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path

use_sim_arg = LaunchArgument("simulation", True, [True, False])


def launch_setup(context: LaunchContext) -> None:
    """Setup the launch description for the navigation.

    Args:
        context (LaunchContext): The launch context.
    """
    use_sim = use_sim_arg.value(context)

    navigation = IncludeLaunchDescription(
        get_file_path("nav2_bringup", ["launch"], "navigation_launch.py"),
        launch_arguments={
            "use_sim_time": str(use_sim),
            "params_file": get_file_path(
                "rcdt_panther", ["config"], "nav2_params.yaml"
            ),
        }.items(),
    )

    group = GroupAction([SetRemap(src="/cmd_vel", dst="/panther/cmd_vel"), navigation])

    return [group]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the navigation.

    Returns:
        LaunchDescription: The launch description for the navigation.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
