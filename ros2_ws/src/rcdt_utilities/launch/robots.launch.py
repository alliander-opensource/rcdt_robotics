# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import SetParameter
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.robot import Robot
from rcdt_utilities.rviz import Rviz

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", True, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
configuration_arg = LaunchArgument(
    "configuration",
    "mm_lidar",
    ["franka", "panther", "panther_lidar", "mm", "mm_lidar"],
)


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Panther robot.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)
    load_gazebo_ui = load_gazebo_ui_arg.bool_value(context)
    use_rviz = use_rviz_arg.bool_value(context)
    configuration = configuration_arg.string_value(context)

    match configuration:
        case "franka":
            Robot("franka", [0, 0, 0])
        case "panther":
            Robot("panther", [0, 0, 0.2])
        case "panther_lidar":
            panther = Robot("panther", [0, 0, 0.2])
            Robot("velodyne", [0.13, -0.13, 0.35], parent=panther)
        case "mm":
            panther = Robot("panther", [0, 0, 0.2])
            Robot("franka", [0, 0, 0.14], parent=panther)
            Robot("velodyne", [0.13, -0.13, 0.35], parent=panther)
        case "mm_lidar":
            panther = Robot("panther", [0, 0, 0.2])
            Robot("franka", [0, 0, 0.14], parent=panther)
            Robot("velodyne", [0.13, -0.13, 0.35], parent=panther)

    state_publishers = Robot.create_state_publishers()
    gazebo = Robot.create_gazebo_launch(load_gazebo_ui)
    tf_publishers = Robot.create_tf_publishers()
    world_links = Robot.create_world_links()
    controllers = Robot.create_controllers()

    Rviz.set_fixed_frame("world")
    Rviz.create_rviz_file()
    rviz = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py")
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        *[Register.on_start(publisher, context) for publisher in state_publishers],
        Register.group(gazebo, context) if use_sim else SKIP,
        *[Register.on_start(tf_publisher, context) for tf_publisher in tf_publishers],
        *[Register.on_start(world_link, context) for world_link in world_links],
        *[Register.group(controller, context) for controller in controllers],
        Register.group(rviz, context) if use_rviz else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Panther robot.

    Returns:
        LaunchDescription: The launch description for the Panther robot.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            use_rviz_arg.declaration,
            configuration_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
