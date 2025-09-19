# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from rcdt_utilities.launch_utils import (
    SKIP,
    LaunchArgument,
    get_file_path,
    get_robot_description,
)
from rcdt_utilities.register import Register

use_sim_arg = LaunchArgument("simulation", True, [True, False])
device_namespace_arg = LaunchArgument("device_namespace", default_value="gps")
robot_namespace_arg = LaunchArgument("robot_namespace", default_value="")


def launch_setup(context: LaunchContext) -> list:
    """Launch the nmea_socket_driver node from the nmea_navsat_driver package.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)
    device_namespace = device_namespace_arg.string_value(context)
    robot_namespace = robot_namespace_arg.string_value(context)

    rename_params_file = ReplaceString(
        source_file=get_file_path(
            "rcdt_panther", ["config"], "nmea_navsat_params.yaml"
        ),
        replacements={"<device_namespace>": str(device_namespace), "//": "/"},
    )

    namespace = "navsat"
    frame_prefix = namespace + "/" if namespace else ""

    xacro_path = get_file_path("rcdt_sensors", ["urdf"], "rcdt_nmea_navsat.urdf.xacro")
    navsat_description = get_robot_description(xacro_path)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="navsat",
        parameters=[navsat_description, {"frame_prefix": frame_prefix}],
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "panther/base_footprint",
            "--child-frame-id",
            "navsat/gps_mounting_point",
            "--x",
            "0.0",
            "--y",
            "-0.06",
            "--z",
            "0.55",
        ],
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

    return [
        Register.on_start(robot_state_publisher, context),
        Register.on_start(static_transform_publisher, context),
        Register.on_start(nmea_driver, context) if not use_sim else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the realsense camera.

    Returns:
        LaunchDescription: The launch description for the realsense camera.
    """
    return LaunchDescription(
        [
            device_namespace_arg.declaration,
            robot_namespace_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
