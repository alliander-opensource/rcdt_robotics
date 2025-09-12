# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription
from rcdt_utilities.robot import Arm, Lidar, Platform, Vehicle
from rcdt_utilities.rviz import Rviz

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", True, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
configuration_arg = LaunchArgument(
    "configuration",
    "mm_lidar",
    ["franka", "panther", "lidar", "panther_lidar", "mm", "mm_lidar"],
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

    Rviz.load_motion_planning_plugin = False
    Rviz.load_point_cloud = False

    match configuration:
        case "franka":
            Arm("franka", [0, 0, 0], moveit=True)
        case "panther":
            Vehicle("panther", [0, 0, 0.2])
        case "lidar":
            Lidar("velodyne", [0, 0, 0.5])
        case "panther_lidar":
            panther = Vehicle("panther", [0, 0, 0.2], navigation=True)
            Lidar("velodyne", [0.13, -0.13, 0.35], parent=panther)
        case "mm":
            panther = Vehicle("panther", [0, 0, 0.2])
            Arm("franka", [0, 0, 0.14], parent=panther)
        case "mm_lidar":
            panther = Vehicle("panther", [0, 0, 0.2], navigation=True)
            Arm("franka", [0, 0, 0.14], parent=panther, moveit=True)
            Lidar("velodyne", [0.13, -0.13, 0.35], parent=panther)

    state_publishers = Platform.create_state_publishers()
    gazebo = Platform.create_gazebo_launch(load_gazebo_ui)
    tf_publishers = Platform.create_tf_publishers()
    world_links = Platform.create_world_links()
    controllers = Platform.create_controllers()
    launch_descriptions = Platform.create_launch_descriptions()

    # Create a tf frame called 'base', required for the MotionPlanning plugin in Rviz:
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "map",
            "--child-frame-id",
            "base",
        ],
    )

    Rviz.set_fixed_frame("map")
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
        *[
            Register.group(launch_description, context)
            for launch_description in launch_descriptions
        ],
        Register.on_start(static_transform_publisher, context)
        if Rviz.load_motion_planning_plugin
        else SKIP,
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
