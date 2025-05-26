# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import (
    SKIP,
    LaunchArgument,
    get_file_path,
    get_robot_description,
)
from rcdt_utilities.register import Register, RegisteredLaunchDescription

use_sim_arg = LaunchArgument("simulation", True, [True, False])
child_arg = LaunchArgument("child", "", ["", "franka"])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "empty_camera.sdf")
use_velodyne_arg = LaunchArgument("velodyne", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    """Setup the launch description for the Panther core.

    Args:
        context (LaunchContext): The launch context.
    """
    use_sim = use_sim_arg.value(context)
    child = child_arg.value(context)
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    world = str(world_arg.value(context))
    use_velodyne = use_velodyne_arg.value(context)

    namespace = "panther"
    frame_prefix = namespace + "/" if namespace else ""
    is_mobile_manipulator = bool(child)

    xacro_path = get_file_path("rcdt_panther", ["urdf"], "panther.urdf.xacro")
    xacro_arguments = {"simulation": "true"}
    xacro_arguments["connected_to"] = child
    xacro_arguments["load_velodyne"] = "true" if use_velodyne else "false"
    robot_description = get_robot_description(xacro_path, xacro_arguments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description, {"frame_prefix": frame_prefix}],
    )

    robot = RegisteredLaunchDescription(
        get_file_path("rcdt_gazebo", ["launch"], "gazebo_robot.launch.py"),
        launch_arguments={
            "world": world,
            "robots": namespace,
            "velodyne": str(use_velodyne),
            "load_gazebo_ui": str(load_gazebo_ui),
        },
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        Register.on_start(robot_state_publisher, context),
        Register.group(robot, context) if not is_mobile_manipulator else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Panther core.

    Returns:
        LaunchDescription: The launch description for the Panther core.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            child_arg.declaration,
            load_gazebo_ui_arg.declaration,
            world_arg.declaration,
            use_velodyne_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
