# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import xml.etree.ElementTree as ET
from typing import List

from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from rcdt_gazebo.gazebo_ros_paths import GazeboRosPaths
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path
from rcdt_utilities.register import Register

load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "empty_camera.sdf")
robots_arg = LaunchArgument("robots", "")
positions_arg = LaunchArgument("positions", "0-0-0")
use_realsense_arg = LaunchArgument("realsense", False, [True, False])
use_velodyne_arg = LaunchArgument("velodyne", False, [True, False])


def launch_setup(context: LaunchContext) -> List:
    """Setup the launch context for the Gazebo simulation with robots.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        List: A list of actions to be executed.
    """
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    world = world_arg.value(context)
    robots = robots_arg.value(context).split(" ")
    positions = positions_arg.value(context).split(" ")
    use_realsense = use_realsense_arg.value(context)
    use_velodyne = use_velodyne_arg.value(context)

    sdf_file = get_file_path("rcdt_gazebo", ["worlds"], world)
    sdf = ET.parse(sdf_file)
    world_name = sdf.getroot().find("world").attrib.get("name")
    cmd = ["ign", "gazebo", sdf_file]
    if not load_gazebo_ui:
        cmd.append("-s")
    gazebo = ExecuteProcess(
        cmd=cmd,
        shell=False,
        additional_env=GazeboRosPaths.get_env(),
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

    spawn_robots: list[Node] = []
    for robot, position in zip(robots, positions):
        namespace = "" if not robot else f"/{robot}"
        x, y, z = position.split("-")
        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic",
                f"{namespace}/robot_description",
                "-x",
                str(x),
                "-y",
                str(y),
                "-z",
                str(z),
            ],
            output="screen",
        )
        spawn_robots.append(spawn_robot)

    unpause_sim = ExecuteProcess(
        cmd=[
            "ign",
            "service",
            "-s",
            f"/world/{world_name}/control",
            "--reqtype",
            "ignition.msgs.WorldControl",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            "pause: false",
        ],
        shell=False,
    )

    return [
        Register.on_start(gazebo, context),
        Register.on_log(bridge, "Creating GZ->ROS Bridge", context),
        *[Register.on_exit(spawn_robot, context) for spawn_robot in spawn_robots],
        Register.on_start(unpause_sim, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Gazebo simulation with robots.

    Returns:
        LaunchDescription: The launch description containing the actions to be executed.
    """
    return LaunchDescription(
        [
            load_gazebo_ui_arg.declaration,
            world_arg.declaration,
            robots_arg.declaration,
            positions_arg.declaration,
            use_realsense_arg.declaration,
            use_velodyne_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
