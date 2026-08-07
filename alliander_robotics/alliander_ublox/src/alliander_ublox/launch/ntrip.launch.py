# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


# From: https://github.com/aussierobots/ublox_dgnss/blob/main/ublox_dgnss/launch/ntrip_client.launch.py

from alliander_utilities.config_objects import GPS
from alliander_utilities.launch_argument import LaunchArgument
from alliander_utilities.register import Register
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

platform_arg = LaunchArgument("platform_config", "")


def launch_setup(context: LaunchContext) -> list:
    """The launch setup.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: The actions to start.
    """
    gps_config = GPS.from_str(platform_arg.string_value(context))
    ntrip_config = gps_config.ntrip_config

    params = [
        {
            "use_https": ntrip_config.use_https,
            "host": ntrip_config.host,
            "port": ntrip_config.port,
            "mountpoint": ntrip_config.mountpoint,
            "username": ntrip_config.username,
            "password": ntrip_config.password,
            "maxage_conn": 30,
        }
    ]

    container_ntrip = ComposableNodeContainer(
        name="ntrip_client_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "INFO"],
        composable_node_descriptions=[
            ComposableNode(
                package="ntrip_client_node",
                plugin="ublox_dgnss::NTRIPClientNode",
                name="ntrip_client",
                parameters=params,
            )
        ],
    )

    return [
        Register.on_start(container_ntrip, context),
    ]


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
