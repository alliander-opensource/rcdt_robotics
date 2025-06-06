# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import SetRemap
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription

use_sim_arg = LaunchArgument("simulation", True, [True, False])


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the navigation.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)

    navigation = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "nav2.launch.py"),
        launch_arguments={
            "use_sim_time": str(use_sim),
            "params_file": get_file_path(
                "rcdt_panther", ["config"], "nav2_params.yaml"
            ),
        },
    )

    return [
        SetRemap(src="/cmd_vel", dst="/panther/cmd_vel"),
        Register.group(navigation, context),
    ]


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
