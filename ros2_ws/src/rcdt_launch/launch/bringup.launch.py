# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rcdt_utilities.launch_utils_env_configuration as utils_config
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import SetParameter
from rcdt_launch.environment_config import EnvironmentConfig
from rcdt_launch.platform_configurations import PLATFORM_CONFIGS, configure_system
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
    list(PLATFORM_CONFIGS.keys()),
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
    load_gazebo_ui = load_gazebo_ui_arg.bool_value(context)
    use_rviz = use_rviz_arg.bool_value(context)
    use_vizanti = use_vizanti_arg.bool_value(context)

    EnvironmentConfig.config_name = configuration_arg.string_value(context)
    EnvironmentConfig.simulation = use_sim_arg.bool_value(context)
    Rviz.load_motion_planning_plugin = False
    Rviz.load_point_cloud = False

    configure_system()

    if EnvironmentConfig.platforms == []:
        raise RuntimeError("No platforms specified. Please specify a platform.")
    utils_config.order_platforms()

    state_publishers = utils_config.create_state_publishers()
    gazebo = utils_config.create_gazebo_launch(load_gazebo_ui)
    hardware_interfaces = utils_config.create_hardware_interfaces()
    map_links = utils_config.create_map_links()
    parent_links = utils_config.create_parent_links()
    controllers = utils_config.create_controllers()
    launch_descriptions = utils_config.create_launch_descriptions()
    joystick_nodes = (
        utils_config.create_joystick_nodes() if EnvironmentConfig.use_joystick else []
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
        SetParameter(name="use_sim_time", value=EnvironmentConfig.simulation),
        Register.group(rviz, context) if use_rviz else SKIP,
        *[Register.on_start(node, context) for node in state_publishers],
        Register.group(gazebo, context) if EnvironmentConfig.simulation else SKIP,
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
