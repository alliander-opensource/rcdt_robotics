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
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
world_arg = LaunchArgument(
    "world", "table_with_1_brick.sdf", ["table_with_1_brick.sdf", "empty_camera.sdf"]
)
use_realsense_arg = LaunchArgument("realsense", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    use_rviz = use_rviz_arg.value(context)
    world = str(world_arg.value(context))
    use_realsense = use_realsense_arg.value(context)

    namespace = "franka"
    ns = f"/{namespace}" if namespace else ""

    core = RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "core.launch.py"),
        launch_arguments={
            "simulation": str(use_sim),
            "load_gazebo_ui": str(load_gazebo_ui),
            "realsense": str(use_realsense),
            "world": world,
        },
    )

    controllers = RegisteredLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "controllers.launch.py")
    )

    display_config = "franka_general.rviz"
    rviz = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={
            "rviz_frame": f"{ns}/fr3_link0",
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
            "namespace": namespace,
        },
    )

    realsense = RegisteredLaunchDescription(
        get_file_path("rcdt_detection", ["launch"], "realsense.launch.py"),
        launch_arguments={"simulation": str(use_sim)},
    )

    joystick = RegisteredLaunchDescription(
        get_file_path("rcdt_joystick", ["launch"], "joystick.launch.py"),
        launch_arguments={"robots": "franka"},
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
        Register.group(controllers, context),
        Register.group(gripper_services, context),
        Register.group(moveit, context),
        Register.group(joystick, context),
        Register.group(utilities, context),
        Register.group(rviz, context) if use_rviz else SKIP,
        Register.group(realsense, context) if use_realsense else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            use_rviz_arg.declaration,
            world_arg.declaration,
            use_realsense_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
