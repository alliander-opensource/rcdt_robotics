# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "empty_camera.sdf")

FRANKA_HEIGHT = 0.34


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the mobile manipulator core.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)
    load_gazebo_ui = load_gazebo_ui_arg.bool_value(context)
    world = world_arg.string_value(context)

    franka_core = RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "core.launch.py"),
        launch_arguments={"parent": "panther", "simulation": str(use_sim)},
    )

    panther_core = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "core.launch.py"),
        launch_arguments={
            "child": "franka",
            "simulation": str(use_sim),
        },
    )

    robots = [
        "franka",
        "panther",
    ]
    positions = [
        f"0-0-{FRANKA_HEIGHT}",
        "0-0-0.2",
    ]
    robot = RegisteredLaunchDescription(
        get_file_path("rcdt_gazebo", ["launch"], "gazebo_robot.launch.py"),
        launch_arguments={
            "world": world,
            "load_gazebo_ui": str(load_gazebo_ui),
            "robots": " ".join(robots),
            "positions": " ".join(positions),
        },
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world",
        arguments=[
            "--frame-id",
            "/panther/base_footprint",
            "--child-frame-id",
            "/franka/fr3_link0",
            "--z",
            f"{FRANKA_HEIGHT}",
        ],
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        Register.group(franka_core, context),
        Register.group(panther_core, context),
        Register.group(robot, context) if use_sim else SKIP,
        Register.on_log(static_transform_publisher, "publishing transform", context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the mobile manipulator core.

    Returns:
        LaunchDescription: The launch description for the mobile manipulator core.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            world_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
