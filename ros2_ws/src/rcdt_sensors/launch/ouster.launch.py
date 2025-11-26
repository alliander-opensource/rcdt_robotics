# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import LifecycleNode
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
    driver_node_name = "ouster_driver"

    sensor_frame = f"{namespace}/ouster"
    lidar_frame = f"{namespace}/os_lidar"
    imu_frame = f"{namespace}/os_imu"

    ouster_driver_node = LifecycleNode(
        package="ouster_ros",
        executable="os_driver",
        namespace=namespace,
        name=driver_node_name,
        parameters=[
            {
                "sensor_hostname": "169.254.233.152",
                "udp_dest": "169.254.89.132",
                "lidar_port": 7502,
                "imu_port": 7503,
                "lidar_mode": "1024x10",  # options: { 512x10, 512x20, 1024x10, 1024x20, 2048x10, 4096x5 }
                "sensor_frame": sensor_frame,
                "lidar_frame": lidar_frame,
                "imu_frame": imu_frame,
                "point_cloud_frame": lidar_frame,
                "metadata": "/tmp/ouster_metadata.json",  # Place the metadata in a temporary folder since we do not need it.
            }
        ],
        output="both",
    )

    configure_ouster_driver = ExecuteProcess(
        cmd=[
            "ros2",
            "lifecycle",
            "set",
            f"/{namespace}/{driver_node_name}",
            "configure",
        ],
        shell=False,
    )

    activate_ouster_driver = ExecuteProcess(
        cmd=[
            "ros2",
            "lifecycle",
            "set",
            f"/{namespace}/{driver_node_name}",
            "activate",
        ],
        shell=False,
    )

    return [
        Register.on_start(ouster_driver_node, context) if not use_sim else SKIP,
        Register.on_exit(configure_ouster_driver, context) if not use_sim else SKIP,
        Register.on_exit(activate_ouster_driver, context) if not use_sim else SKIP,
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
