# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from alliander_utilities.adapted_yaml import AdaptedYaml
from alliander_utilities.config_objects import GPS
from alliander_utilities.launch_argument import LaunchArgument
from alliander_utilities.register import Register
from alliander_utilities.ros_utils import get_file_path
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

platform_arg = LaunchArgument("platform_config", "")


def launch_rover(context: LaunchContext, params: AdaptedYaml, namespace: str) -> list:
    """Launches a standalone rover configuration.

    Args:
        context (LaunchContext): The current launch context.
        params (AdaptedYaml): Parameters to send to the U-blox chips.
        namespace (str): The GPS namespace.

    Returns:
       list: The list of nodes to launch.
    """
    container_rover = ComposableNodeContainer(
        name="ublox_dgnss_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "INFO"],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                namespace=namespace,
                parameters=[params.file],
            )
        ],
    )

    container_navsatfix = ComposableNodeContainer(
        name="ublox_nav_sat_fix_hp_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "INFO"],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_nav_sat_fix_hp_node",
                plugin="ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode",
                name="ublox_nav_sat_fix_hp",
                namespace=namespace,
                remappings=[("fix", "gps/fix")],
            )
        ],
    )

    return [
        Register.on_start(container_rover, context),
        Register.on_start(container_navsatfix, context),
    ]


def launch_fb_base(context: LaunchContext, params: AdaptedYaml, namespace: str) -> list:
    """Launches a fixed base configuration which receives NTRIP corrections and sends them to a rover.

    Args:
        context (LaunchContext): The current launch context.
        params (AdaptedYaml): Parameters to send to the U-blox chips.
        namespace (str): The GPS namespace.

    Returns:
       list: The list of nodes to launch.
    """
    container_base = ComposableNodeContainer(
        name="ublox_dgnss_base",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "INFO"],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                namespace=namespace,
                parameters=[params.file],
            )
        ],
    )
    return [
        Register.on_start(container_base, context),
    ]


def launch_fb_rover(
    context: LaunchContext, params: AdaptedYaml, namespace: str
) -> list:
    """Launches a rover configuration to be used in conjunction with a fixed base, which is on the same ROS network.

    Args:
        context (LaunchContext): The current launch context.
        params (AdaptedYaml): Parameters to send to the U-blox chips.
        namespace (str): The GPS namespace.

    Returns:
       list: The list of nodes to launch.
    """
    print(f"Params file: {params.file}")
    container_rover = ComposableNodeContainer(
        name="ublox_dgnss_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "INFO"],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                namespace=namespace,
                parameters=[params.file],
                remappings=[("/ntrip_client/rtcm", "/base/rtcm")],
            )
        ],
    )

    container_navsatfix = ComposableNodeContainer(
        name="ublox_nav_sat_fix_hp_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "INFO"],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_nav_sat_fix_hp_node",
                plugin="ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode",
                name="ublox_nav_sat_fix_hp",
                namespace=namespace,
            )
        ],
    )
    return [
        Register.on_start(container_rover, context),
        Register.on_start(container_navsatfix, context),
    ]


def launch_mb_base(context: LaunchContext, params: AdaptedYaml, namespace: str) -> list:
    """Launches a moving base configuration that sends differential corrections to a rover over UART.

    Args:
        context (LaunchContext): The current launch context.
        params (AdaptedYaml): Parameters to send to the U-blox chips.
        namespace (str): The GPS namespace.

    Returns:
       list: The list of nodes to launch.
    """
    container_base = ComposableNodeContainer(
        name="ublox_dgnss_moving_base",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "INFO"],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                namespace=namespace,
                parameters=[params.file],
            )
        ],
    )

    container_navsatfix = ComposableNodeContainer(
        name="ublox_nav_sat_fix_hp_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "INFO"],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_nav_sat_fix_hp_node",
                plugin="ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode",
                namespace=namespace,
                name="ublox_nav_sat_fix_hp",
            )
        ],
    )
    return [
        Register.on_start(container_base, context),
        Register.on_start(container_navsatfix, context),
    ]


def launch_mb_rover(
    context: LaunchContext, params: AdaptedYaml, namespace: str
) -> list:
    """Launches a rover configuration to be used in conjunction with a moving base that sends the rover differential RTK corrections.

    Args:
        context (LaunchContext): The current launch context.
        params (AdaptedYaml): Parameters to send to the U-blox chips.
        namespace (str): The GPS namespace.

    Returns:
       list: The list of nodes to launch.
    """
    container_rover = ComposableNodeContainer(
        name="ublox_dgnss_rover",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "INFO"],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                namespace=namespace,
                parameters=[params.file],
            )
        ],
    )

    container_navsatfix = ComposableNodeContainer(
        name="ublox_nav_sat_fix_hp_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "INFO"],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_nav_sat_fix_hp_node",
                plugin="ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode",
                namespace=namespace,
                name="ublox_nav_sat_fix_hp",
            )
        ],
    )

    return [
        Register.on_start(container_rover, context),
        Register.on_start(container_navsatfix, context),
    ]


def launch_setup(context: LaunchContext) -> list:
    """The launch setup.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: The actions to start.
    """
    gps_config = GPS.from_str(platform_arg.string_value(context))
    params = AdaptedYaml(
        get_file_path(
            "alliander_ublox", ["config"], f"{gps_config.operation_mode}.yaml"
        ),
        {
            "DEVICE_FAMILY": gps_config.device_family,
            "DEVICE_SERIAL_STRING": gps_config.usb_device,
            "FRAME_ID": f"{gps_config.namespace}/base_link",
        },
        root_key=gps_config.namespace,
    )

    match gps_config.operation_mode:
        case "rover":
            return launch_rover(context, params, gps_config.namespace)
        case "fb_base":
            return launch_fb_base(context, params, gps_config.namespace)
        case "fb_rover":
            return launch_fb_rover(context, params, gps_config.namespace)
        case "mb_base":
            return launch_mb_base(context, params, gps_config.namespace)
        case "mb_rover":
            return launch_mb_rover(context, params, gps_config.namespace)
        case _:
            print(
                f"WARNING: invalid configuration specified ({gps_config.operation_mode})."
            )
            return []


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description.

    Returns:
        LaunchDescription: The launch description.
    """
    return LaunchDescription(
        [
            platform_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
