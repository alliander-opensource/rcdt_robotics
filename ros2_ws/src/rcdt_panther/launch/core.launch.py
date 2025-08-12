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
child_arg = LaunchArgument("child", "", ["", "franka", "velodyne"])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "empty_camera.sdf")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Panther core.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)
    child = child_arg.string_value(context)
    load_gazebo_ui = load_gazebo_ui_arg.bool_value(context)
    world = world_arg.string_value(context)

    namespace = "panther"
    frame_prefix = namespace + "/" if namespace else ""
    is_mobile_manipulator = child == "franka"

    xacro_path = get_file_path("rcdt_panther", ["urdf"], "panther.urdf.xacro")
    xacro_arguments = {"simulation": "true"}
    xacro_arguments["connected_to"] = child
    robot_description = get_robot_description(xacro_path, xacro_arguments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[robot_description, {"frame_prefix": frame_prefix}],
    )

    robots = ["panther"]
    positions = ["0-0-0"]
    if child == "velodyne":
        robots.append("velodyne")
        positions.append("0-0-0.35")
    robot = RegisteredLaunchDescription(
        get_file_path("rcdt_gazebo", ["launch"], "gazebo_robot.launch.py"),
        launch_arguments={
            "world": world,
            "robots": " ".join(robots),
            "positions": " ".join(positions),
            "load_gazebo_ui": str(load_gazebo_ui),
            "positions": positions,
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
            OpaqueFunction(function=launch_setup),
        ]
    )
