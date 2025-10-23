# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path
from rcdt_utilities.register import Register

use_sim_arg = LaunchArgument("simulation", True, [True, False])
namespace_arg = LaunchArgument("namespace", "ouster")
target_frame_arg = LaunchArgument("target_frame", "")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the realsense camera.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed.
    """
    use_sim = use_sim_arg.bool_value(context)
    namespace = namespace_arg.string_value(context)
    target_frame = target_frame_arg.string_value(context)

    frame_prefix = namespace + "/" if namespace else ""

    ouster_driver_node = None  # Insert Ouster driver node

    ouster_transform_node = None  # Insert Ouster transform pointcloud node

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[
            ("cloud_in", f"/{namespace}/scan/points"),
            ("scan", f"/{namespace}/scan"),
        ],
        parameters=[
            {
                "target_frame": target_frame,
                "min_height": 0.1,
                "max_height": 2.0,
                "range_min": 0.05,
                "range_max": 90.0,
            }
        ],
        namespace=namespace,
    )

    return [
        # Register.on_start(ouster_driver_node, context) if not use_sim else SKIP,
        # Register.on_start(ouster_transform_node, context) if not use_sim else SKIP,
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
            namespace_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
