# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from rcdt_utilities.launch_utils import get_file_path


def generate_launch_description() -> LaunchDescription:
    """Launch the nmea_socket_driver node from the nmea_navsat_driver package.

    Returns:
        LaunchDescription: The launch description containing the nmea_socket_driver node.
    """
    device_namespace = LaunchConfiguration("device_namespace")
    declare_device_namespace_arg = DeclareLaunchArgument(
        "device_namespace",
        default_value="gps",
        description="Namespace for the device, utilized in TF frames and preceding device topics. This aids in differentiating between multiple cameras on the same robot.",
    )

    params_file = LaunchConfiguration("params_file")
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=get_file_path(
            "rcdt_panther", ["config"], "nmea_navsat_params.yaml"
        ),
        description="Path to the parameter file for the nmea_socket_driver node.",
    )

    robot_namespace = LaunchConfiguration("robot_namespace")
    declare_robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace to all launched nodes and use namespace as tf_prefix. This aids in differentiating between multiple robots with the same devices.",
    )

    rename_params_file = ReplaceString(
        source_file=params_file,
        replacements={"<device_namespace>": device_namespace, "//": "/"},
    )

    nmea_driver = Node(
        package="nmea_navsat_driver",
        executable="nmea_socket_driver",
        name=device_namespace,
        namespace=robot_namespace,
        parameters=[
            {
                "frame_id": device_namespace,
                "tf_prefix": robot_namespace,
            },
            rename_params_file,
        ],
        remappings=[
            ("fix", "~/fix"),
            ("heading", "~/heading"),
            ("time_reference", "~/time_reference"),
            ("vel", "~/vel"),
        ],
    )

    return LaunchDescription(
        [
            declare_params_file_arg,
            declare_robot_namespace_arg,
            declare_device_namespace_arg,
            nmea_driver,
        ]
    )
