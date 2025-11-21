# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import LifecycleNode, Node
from rcdt_utilities.launch_argument import LaunchArgument
from rcdt_utilities.launch_utils import SKIP
from rcdt_utilities.register import Register

use_sim_arg = LaunchArgument("simulation", True, [True, False])
namespace_arg = LaunchArgument("namespace", "ouster")
target_frame_arg = LaunchArgument("target_frame", "")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Ouster lidar.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed.
    """
    use_sim = use_sim_arg.bool_value(context)
    namespace = namespace_arg.string_value(context)
    target_frame = target_frame_arg.string_value(context)

    ouster_driver_node = LifecycleNode(
        package="ouster_ros",
        executable="os_driver",
        namespace=namespace,
        name="ouster_driver",
        parameters=[
            {
                "sensor_hostname": "169.254.233.152",
                "udp_dest": "169.254.89.132",
                "lidar_port": 7502,
                "imu_port": 7503,
                "lidar_mode": "1024x10",  # possible values: { 512x10, 512x20, 1024x10, 1024x20, 2048x10, 4096x5 }
            }
        ],
        output="both",
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_ouster",
        output="screen",
        parameters=[{"autostart": True}, {"node_names": ["ouster_driver"]}],
        namespace=namespace,
        remappings=[
            # in-cloud: ("cloud_in", f"/{namespace}/scan/points"),
            # the output scan: ("scan", f"/{namespace}/scan"),
            # TODO: placeholder for possibly required remappings.
        ],
    )

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
        Register.on_start(ouster_driver_node, context) if not use_sim else SKIP,
        Register.on_log(lifecycle_manager, "Managed ouster nodes are active", context)
        if not use_sim
        else SKIP,
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
