# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Panther controllers.

    Returns:
        LaunchDescription: The launch description containing the Panther controllers.
    """

    namespace = os.environ.get("NAMESPACE", default="panther")
    simulation = os.environ.get("SIMULATION", default="False")
    slam = os.environ.get("SLAM", default="False")
    collision_monitor = os.environ.get("COLLISION_MONITOR", default="False")
    navigation = os.environ.get("NAVIGATION", default="False")
    use_gps = os.environ.get("USE_GPS", default="False")
    window_size = os.environ.get("WINDOW_SIZE", default="10")

    pkg_dir = get_package_share_directory("rcdt_husarion")

    controller_launch_file = os.path.join(pkg_dir, "launch", "controllers.launch.py")
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_file)
    )

    nav2_launch_file = os.path.join(pkg_dir, "launch", "nav2.launch.py")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "simulation": simulation,
            "slam": slam,
            "collision_monitor": collision_monitor,
            "navigation": navigation,
            "namespace_vehicle": namespace,
            "namespace_lidar": namespace,
            "namespace_gps": namespace,
            "use_gps": use_gps,
            "window_size": window_size,
        }.items(),
    )

    utils_launch_file = os.path.join(pkg_dir, "launch", "utils.launch.py")
    utils_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(utils_launch_file)
    )

    return LaunchDescription(
        [
            # controller_launch,
            # nav2_launch,
            utils_launch,
        ]
    )
