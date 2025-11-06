# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import itertools

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import SKIP, LaunchArgument
from rcdt_utilities.register import Register

use_sim_arg = LaunchArgument("simulation", True, [True, False])
namespace_arg = LaunchArgument("namespace", default_value="")
ip_address_arg = LaunchArgument("ip_address", "")

T = True
F = False

# Parameters based on https://github.com/ros-navigation/navigation2_tutorials/blob/rolling/nav2_gps_waypoint_follower_demo/config/dual_ekf_navsat_params.yaml


def launch_setup(context: LaunchContext) -> list:
    """Launch the nmea_socket_driver node from the nmea_navsat_driver package.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)
    namepace = namespace_arg.string_value(context)
    ip_address = ip_address_arg.string_value(context)

    nmea_driver = Node(
        package="nmea_navsat_driver",
        executable="nmea_socket_driver",
        name="gps",
        namespace=namepace,
        parameters=[
            {
                "ip": ip_address,
                "port": 5000,
                "frame_id": "gps",
                "tf_prefix": namepace,
            },
        ],
        remappings=[
            ("fix", "~/fix"),
            ("heading", "~/heading"),
            ("time_reference", "~/time_reference"),
            ("vel", "~/vel"),
        ],
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        namespace=namepace,
        parameters=[
            {
                "publish_filtered_gps": False,
            }
        ],
        remappings=[("imu", "/panther1/imu/data")],
    )

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace=namepace,
        parameters=[
            {
                "two_d_mode": True,
                "publish_tf": True,
                "world_frame": "panther1/odom",
                "map_frame": "map",
                "odom_frame": "panther1/odom",
                "base_link_frame": "panther1/base_link",
                "odom0": "/panther1/odometry/wheels",
                "odom0_config": list(
                    itertools.chain.from_iterable(
                        [
                            [F, F, F],  # [x_pos, y_pos, z_pos]
                            [F, F, F],  # [roll, pitch, yaw]
                            [T, T, T],  # [x_vel, y_vel, z_vel]
                            [F, F, T],  # [roll_rate, pitch_rate, yaw_rate]
                            [F, F, F],  # [x_accel, y_accel, z_accel]
                        ]
                    )
                ),
            }
        ],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace=namepace,
        parameters=[
            {
                "two_d_mode": True,
                "publish_tf": True,
                "world_frame": "map",
                "map_frame": "map",
                "odom_frame": "panther1/odom",
                "base_link_frame": "panther1/base_footprint",
                "odom0": "/panther1/odometry/wheels",
                "odom0_config": list(
                    itertools.chain.from_iterable(
                        [
                            [F, F, F],  # [x_pos, y_pos, z_pos]
                            [F, F, F],  # [roll, pitch, yaw]
                            [T, T, T],  # [x_vel, y_vel, z_vel]
                            [F, F, T],  # [roll_rate, pitch_rate, yaw_rate]
                            [F, F, F],  # [x_accel, y_accel, z_accel]
                        ]
                    )
                ),
                "odom1": "/nmea1/odometry/gps",
                "odom1_config": list(
                    itertools.chain.from_iterable(
                        [
                            [T, T, F],  # [x_pos, y_pos, z_pos]
                            [F, F, F],  # [roll, pitch, yaw]
                            [F, F, F],  # [x_vel, y_vel, z_vel]
                            [F, F, F],  # [roll_rate, pitch_rate, yaw_rate]
                            [F, F, F],  # [x_accel, y_accel, z_accel]
                        ]
                    )
                ),
                "imu0": "/panther1/imu/data",
                "imu0_config": list(
                    itertools.chain.from_iterable(
                        [
                            [F, F, F],  # [x_pos, y_pos, z_pos]
                            [F, F, T],  # [roll, pitch, yaw]
                            [F, F, F],  # [x_vel, y_vel, z_vel]
                            [F, F, F],  # [roll_rate, pitch_rate, yaw_rate]
                            [F, F, F],  # [x_accel, y_accel, z_accel]
                        ]
                    )
                ),
            }
        ],
    )

    return [
        Register.on_start(nmea_driver, context) if not use_sim else SKIP,
        Register.on_start(navsat_transform_node, context),
        Register.on_start(ekf_local, context),
        Register.on_start(ekf_global, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the realsense camera.

    Returns:
        LaunchDescription: The launch description for the realsense camera.
    """
    return LaunchDescription(
        [
            namespace_arg.declaration,
            ip_address_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
