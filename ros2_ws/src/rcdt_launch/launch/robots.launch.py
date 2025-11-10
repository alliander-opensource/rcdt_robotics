# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import SetParameter
from rcdt_launch.platform_configurations import (
    PLATFORM_CONFIGS,
    ConfigurationContext,
    configure_system,
)
from rcdt_launch.robot import Platform
from rcdt_launch.rviz import Rviz
from rcdt_launch.vizanti import Vizanti
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
use_vizanti_arg = LaunchArgument("vizanti", False, [True, False])
configuration_arg = LaunchArgument(
    "configuration",
    "",
    [""] + list(PLATFORM_CONFIGS.keys()),
)


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Panther robot.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.

    Raises:
        RuntimeError: If no platforms are specified.
    """
    platform_ctx: ConfigurationContext = ConfigurationContext(
        config_namespace=configuration_arg.string_value(context),
        use_joystick=True,
        use_sim=use_sim_arg.bool_value(context),
    )

    load_gazebo_ui = load_gazebo_ui_arg.bool_value(context)
    use_rviz = use_rviz_arg.bool_value(context)
    use_vizanti = use_vizanti_arg.bool_value(context)

    Platform.simulation = platform_ctx.use_sim
    Rviz.load_motion_planning_plugin = False
    Rviz.load_point_cloud = False

    platform_ctx = configure_system(platform_ctx)

    if Platform.platforms == []:
        raise RuntimeError("No platforms specified. Please specify a platform.")
    Platform.order_platforms()

    state_publishers = Platform.create_state_publishers()
    gazebo = Platform.create_gazebo_launch(load_gazebo_ui)
    hardware_interfaces = Platform.create_hardware_interfaces()
    map_links = Platform.create_map_links()
    parent_links = Platform.create_parent_links()
    controllers = Platform.create_controllers()
    launch_descriptions = Platform.create_launch_descriptions()
    joystick_nodes = (
        Platform.create_joystick_nodes() if platform_ctx.use_joystick else []
    )

    utilities = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "utils.launch.py")
    )

    Rviz.set_fixed_frame("map")
    Rviz.create_rviz_file()
    rviz = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py")
    )

    Vizanti.create_config_file()
    vizanti = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "vizanti.launch.py")
    )

    return [
        SetParameter(name="use_sim_time", value=platform_ctx.use_sim),
        Register.group(rviz, context) if use_rviz else SKIP,
        *[Register.on_start(node, context) for node in state_publishers],
        Register.group(gazebo, context) if platform_ctx.use_sim else SKIP,
        *[Register.group(group, context) for group in hardware_interfaces],
        *[Register.on_start(node, context) for node in map_links],
        *[Register.on_start(node, context) for node in parent_links],
        *[Register.group(node, context) for node in controllers],
        Register.group(utilities, context),
        *[Register.on_start(node, context) for node in joystick_nodes],
        *[Register.group(group, context) for group in launch_descriptions],
        Register.group(vizanti, context) if use_vizanti else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Panther robot.

    Returns:
        LaunchDescription: The launch description for the Panther robot.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            use_rviz_arg.declaration,
            use_vizanti_arg.declaration,
            configuration_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
