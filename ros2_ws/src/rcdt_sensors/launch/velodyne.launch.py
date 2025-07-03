# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from numpy import deg2rad
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path
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

    namespace = "panther"
    device = "velodyne"
    frame_prefix = namespace + "/" if namespace else ""

    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        output="both",
        parameters=[
            {
                "device_ip": "10.15.20.5",
                "frame_id": frame_prefix + "velodyne",
            }
        ],
        namespace=namespace + "/" + device,
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
                "min_range": 0.9,
                "max_range": 130.0,
            }
        ],
        namespace=namespace + "/" + device,
    )

    velodyne_laserscan_node = Node(
        package="velodyne_laserscan",
        executable="velodyne_laserscan_node",
        output="both",
        parameters=[
            {
                "ring": -1,
                "resolution": 0.007,
            }
        ],
        namespace=namespace + "/" + device,
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            frame_prefix + "cover_link",
            "--child-frame-id",
            frame_prefix + "velodyne",
            "--x",
            "0.2",
            "--yaw",
            str(deg2rad(90)),
        ],
    )

    return [
        Register.on_start(velodyne_driver_node, context),
        Register.on_start(velodyne_transform_node, context),
        Register.on_start(velodyne_laserscan_node, context),
        Register.on_start(static_transform_publisher, context),
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
