# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path

load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "empty_camera.sdf")
namespace_arg = LaunchArgument("namespace", "")
use_realsense_arg = LaunchArgument("realsense", False, [True, False])
use_velodyne_arg = LaunchArgument("velodyne", False, [True, False])


def launch_setup(context: LaunchContext) -> List:
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    world = world_arg.value(context)
    namespace = namespace_arg.value(context)
    use_realsense = use_realsense_arg.value(context)
    use_velodyne = use_velodyne_arg.value(context)

    sdf_file = get_file_path("rcdt_gz_worlds", ["worlds"], world)
    gz_args = f" -r {sdf_file}"
    if not load_gazebo_ui:
        gz_args += " -s"
    gazebo = IncludeLaunchDescription(
        get_file_path("ros_gz_sim", ["launch"], "gz_sim.launch.py"),
        launch_arguments={"gz_args": gz_args}.items(),
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
                "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
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
