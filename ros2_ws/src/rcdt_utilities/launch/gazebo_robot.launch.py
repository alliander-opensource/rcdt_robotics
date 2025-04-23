# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import os
from os import environ
from typing import List

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import (
    PACKAGE_MANIFEST_FILENAME,
    Export,
    InvalidPackage,
    Package,
    parse_package,
)
from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path
from ros2pkg.api import get_package_names

load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "empty_camera.sdf")
namespace_arg = LaunchArgument("namespace", "")
use_realsense_arg = LaunchArgument("realsense", False, [True, False])
use_velodyne_arg = LaunchArgument("velodyne", False, [True, False])


class GazeboRosPaths:
    """
    Based on: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_sim.launch.py.in.

    By copying instead of using that launch file we are able to launch Gazebo as process with shell=False.
    This avoids ghost processes of Gazebo continuing and might be implemented in the original launch file in the future:
    https://github.com/gazebosim/ros_gz/issues/563
    """

    @staticmethod
    def get_paths() -> tuple[str, str]:
        gazebo_model_path = []
        gazebo_plugin_path = []
        gazebo_media_path = []

        for package_name in get_package_names():
            package_share_path = get_package_share_directory(package_name)
            package_file_path = os.path.join(
                package_share_path, PACKAGE_MANIFEST_FILENAME
            )
            if os.path.isfile(package_file_path):
                try:
                    package: Package = parse_package(package_file_path)
                except InvalidPackage:
                    continue
                for export in package.exports:
                    export: Export
                    if export.tagname == "gazebo_ros":
                        if "gazebo_model_path" in export.attributes:
                            xml_path = export.attributes["gazebo_model_path"]
                            xml_path: str = xml_path.replace(
                                "${prefix}", package_share_path
                            )
                            gazebo_model_path.append(xml_path)
                        if "plugin_path" in export.attributes:
                            xml_path = export.attributes["plugin_path"]
                            xml_path = xml_path.replace("${prefix}", package_share_path)
                            gazebo_plugin_path.append(xml_path)
                        if "gazebo_media_path" in export.attributes:
                            xml_path = export.attributes["gazebo_media_path"]
                            xml_path = xml_path.replace("${prefix}", package_share_path)
                            gazebo_media_path.append(xml_path)

        gazebo_model_path = os.pathsep.join(gazebo_model_path + gazebo_media_path)
        gazebo_plugin_path = os.pathsep.join(gazebo_plugin_path)

        return gazebo_model_path, gazebo_plugin_path

    @staticmethod
    def get_env() -> dict[str, str]:
        model_paths, plugin_paths = GazeboRosPaths.get_paths()
        env = {
            "GZ_SIM_SYSTEM_PLUGIN_PATH": os.pathsep.join(
                [
                    environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", default=""),
                    environ.get("LD_LIBRARY_PATH", default=""),
                    plugin_paths,
                ]
            ),
            "GZ_SIM_RESOURCE_PATH": os.pathsep.join(
                [
                    environ.get("GZ_SIM_RESOURCE_PATH", default=""),
                    model_paths,
                ]
            ),
        }
        return env


def launch_setup(context: LaunchContext) -> List:
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    world = world_arg.value(context)
    namespace = namespace_arg.value(context)
    use_realsense = use_realsense_arg.value(context)
    use_velodyne = use_velodyne_arg.value(context)

    sdf_file = get_file_path("rcdt_gz_worlds", ["worlds"], world)
    cmd = ["ign", "gazebo", sdf_file, "-r"]
    if not load_gazebo_ui:
        cmd.append("-s")
    gazebo = ExecuteProcess(
        cmd=cmd,
        shell=False,
        additional_env=GazeboRosPaths.get_env(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", f"{namespace}/robot_description"],
        output="screen",
    )

    bridge_topics = ["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"]
    if use_realsense:
        bridge_topics.extend(
            [
                "/camera/camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                "/camera/camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
                "/camera/camera/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                "/camera/camera/depth/image_rect_raw_float@sensor_msgs/msg/Image@gz.msgs.Image",
            ]
        )
    if use_velodyne:
        bridge_topics.extend(
            [
                "/panther/velodyne/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                "/panther/velodyne/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            ]
        )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_topics,
    )

    return [
        gazebo,
        spawn_robot,
        bridge,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            load_gazebo_ui_arg.declaration,
            world_arg.declaration,
            namespace_arg.declaration,
            use_realsense_arg.declaration,
            use_velodyne_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
