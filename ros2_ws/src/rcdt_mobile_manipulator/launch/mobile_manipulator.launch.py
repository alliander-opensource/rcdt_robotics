# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import SetParameter
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "empty_camera.sdf")
use_rviz_arg = LaunchArgument("rviz", True, [True, False])


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the mobile manipulator.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)
    load_gazebo_ui = load_gazebo_ui_arg.bool_value(context)
    use_rviz = use_rviz_arg.bool_value(context)
    world = world_arg.string_value(context)

    core = RegisteredLaunchDescription(
        get_file_path("rcdt_mobile_manipulator", ["launch"], "core.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "load_gazebo_ui": str(load_gazebo_ui),
            "world": world,
        },
    )

    franka_controllers = RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "controllers.launch.py"),
        launch_arguments={"simulation": str(use_sim)},
    )

    panther_controllers = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "controllers.launch.py"),
    )

    display_config = "mobile_manipulator_general.rviz"
    rviz = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={
            "rviz_frame": "panther/odom",
            "robot_name": "fr3",
            "moveit_package_name": "rcdt_franka_moveit_config",
            "rviz_display_config": display_config,
        },
    )

    moveit = RegisteredLaunchDescription(
        get_file_path("rcdt_moveit", ["launch"], "moveit.launch.py"),
        launch_arguments={
            "robot_name": "fr3",
            "moveit_package_name": "rcdt_franka_moveit_config",
            "servo_params_package": "rcdt_franka",
            "namespace": "franka",
        },
    )

    joystick = RegisteredLaunchDescription(
        get_file_path("rcdt_joystick", ["launch"], "joystick.launch.py"),
        launch_arguments={"simulation": str(use_sim), "robots": "franka panther"},
    )

    gripper_services = RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "gripper_services.launch.py")
    )

    utilities = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "utils.launch.py")
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        Register.group(core, context),
        Register.group(franka_controllers, context),
        Register.group(panther_controllers, context) if use_sim else SKIP,
        Register.group(gripper_services, context),
        Register.group(moveit, context),
        Register.group(joystick, context),
        Register.group(utilities, context),
        Register.group(rviz, context) if use_rviz else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the mobile manipulator.

    Returns:
        LaunchDescription: The launch description for the mobile manipulator.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            world_arg.declaration,
            use_rviz_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
