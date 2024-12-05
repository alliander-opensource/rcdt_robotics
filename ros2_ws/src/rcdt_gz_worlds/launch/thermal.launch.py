# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    OpaqueFunction,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


# Declare launch arguments
world_arg = LaunchArgument("world", default_value="thermal_camera.sdf")


def launch_setup(context: LaunchContext) -> list:
    world = world_arg.value(context)
    # Get the package share directory
    # pkg_share = get_package_share_directory("rcdt_anomaly")

    # # Setting gazebo environment variable for models
    # models_path = SetEnvironmentVariable(
    #     name="IGN_GAZEBO_RESOURCE_PATH",
    #     value=[
    #         os.path.join(pkg_share, "worlds"),
    #         ":" + str(Path(pkg_share).parent.resolve()),
    #     ],
    # )

    # Reference to thermal_camera.sdf
    gazebo = IncludeLaunchDescription(
        get_file_path("ros_gz_sim", ["launch"], "gz_sim.launch.py"),
        launch_arguments={"gz_args": f"{world} -r"}.items(),
    )

    # Bridge needed for /thermal_camera topic
    thermal_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="thermal_camera_bridge",
        arguments=["/thermal_camera@sensor_msgs/msg/Image@ignition.msgs.Image"],
        output="screen",
    )

    # Node for thermal camera analysis file
    thermal_camera_analysis = Node(
        package="rcdt_anomaly",  # Replace with your package name
        executable="thermal_camera_analysis.py",  # Python script name
        name="thermal_camera_analysis",  # Node name
    )

    return [
        # models_path,
        gazebo,
        thermal_camera_bridge,
        thermal_camera_analysis,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            world_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
