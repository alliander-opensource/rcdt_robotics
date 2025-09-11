# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import (
    SKIP,
    LaunchArgument,
    get_file_path,
    get_robot_description,
)
from rcdt_utilities.register import Register

use_sim_arg = LaunchArgument("simulation", True, [True, False])


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the realsense camera.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed.
    """
    use_sim = use_sim_arg.bool_value(context)

    namespace = "velodyne"
    frame_prefix = namespace + "/" if namespace else ""

    xacro_path = get_file_path("rcdt_sensors", ["urdf"], "rcdt_velodyne.urdf.xacro")
    velodyne_description = get_robot_description(xacro_path)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="velodyne",
        parameters=[velodyne_description, {"frame_prefix": frame_prefix}],
    )

    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        output="both",
        parameters=[
            {
                "model": "VLP16",
                "device_ip": "10.15.20.5",
                "frame_id": frame_prefix + "velodyne",
            }
        ],
        namespace=namespace,
    )

    velodyne_transform_node = Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        output="both",
        parameters=[
            {
                "calibration": get_file_path(
                    "velodyne_pointcloud", ["params"], "VLP16db.yaml"
                ),
                "model": "VLP16",
                "min_range": 0.1,
                "max_range": 130.0,
            }
        ],
        remappings=[("velodyne_points", "scan/points")],
        namespace=namespace,
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "panther/base_link",
            "--child-frame-id",
            "velodyne/base_link",
            "--x",
            "0.0",
            "--y",
            "-0.06",
            "--z",
            "0.35",
        ],
    )
    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/velodyne/scan/points"), ("scan", "/velodyne/scan")],
        parameters=[
            {
                "target_frame": "panther/base_footprint",
                "min_height": 0.1,
                "max_height": 2.0,
                "range_min": 0.05,
                "range_max": 100.0,
            }
        ],
    )

    return [
        Register.on_start(robot_state_publisher, context),
        Register.on_start(static_transform_publisher, context),
        Register.on_start(velodyne_driver_node, context) if not use_sim else SKIP,
        Register.on_start(velodyne_transform_node, context) if not use_sim else SKIP,
        Register.on_start(pointcloud_to_laserscan_node, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the realsense camera.

    Returns:
        LaunchDescription: The launch description for the realsense camera.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
