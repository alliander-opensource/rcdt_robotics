import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
)
from launch_ros.actions import Node

from rcdt_utilities.launch_utils import LaunchArgument, get_file_path

load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", True, [True, False])
world_arg = LaunchArgument("world", "table_brick.sdf")


def launch_setup(context: LaunchContext) -> List:
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    world = world_arg.value(context)

    pkg_share = get_package_share_directory("rcdt_gz_worlds")

    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            os.path.join(pkg_share, "worlds"),
            ":" + os.path.join(pkg_share, "models"),
        ],
    )

    print("-------------")
    print(os.path.join(pkg_share, "worlds"), ":" + os.path.join(pkg_share, "models"))

    gz_args = f" -r {world}"
    if not load_gazebo_ui:
        gz_args += " -s"
    gazebo = IncludeLaunchDescription(
        get_file_path("ros_gz_sim", ["launch"], "gz_sim.launch.py"),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
    )

    return [
        ign_resource_path,
        gazebo,
        bridge,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            load_gazebo_ui_arg.declaration,
            world_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
