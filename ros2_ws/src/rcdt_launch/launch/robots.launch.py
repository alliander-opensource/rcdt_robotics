# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import SetParameter
from rcdt_launch.robot import GPS, Arm, Camera, Lidar, Platform, Vehicle
from rcdt_launch.rviz import Rviz
from rcdt_launch.vizanti import Vizanti
from rcdt_utilities.launch_utils import SKIP, LaunchArgument, get_file_path
from rcdt_utilities.register import Register, RegisteredLaunchDescription

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
use_vizanti_arg = LaunchArgument("vizanti", False, [True, False])
configuration_arg = LaunchArgument(
    "configuration",
    "",
    [
        "",
        "axis",
        "gps",
        "ouster",
        "velodyne",
        "realsense",
        "zed",
        "franka",
        "franka_axis",
        "franka_double",
        "franka_realsense",
        "panther",
        "panther_axis",
        "panther_gps",
        "panther_realsense",
        "panther_zed",
        "panther_lidar",
        "panther_ouster",
        "mm",
        "mm_lidar",
        "mm_ouster",
        "panther_and_franka",
    ],
)


def launch_setup(context: LaunchContext) -> list:  # noqa: PLR0912, PLR0915
    """Setup the launch description for the Panther robot.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.

    Raises:
        RuntimeError: If no platforms are specified.
    """
    use_sim = use_sim_arg.bool_value(context)
    load_gazebo_ui = load_gazebo_ui_arg.bool_value(context)
    use_rviz = use_rviz_arg.bool_value(context)
    use_vizanti = use_vizanti_arg.bool_value(context)
    configuration = configuration_arg.string_value(context)

    Platform.simulation = use_sim
    Rviz.load_motion_planning_plugin = False
    Rviz.load_point_cloud = False

    use_joystick = True

    match configuration:
        case "axis":
            Platform("axis", [0, 0, 0])
        case "gps":
            GPS("nmea", [0, 0, 0.5], ip_address="10.15.20.202")
        case "ouster":
            Lidar("ouster", [0, 0, 0.5])
        case "velodyne":
            Lidar("velodyne", [0, 0, 0.5])
        case "realsense":
            Camera("realsense", [0, 0, 0.5])
        case "zed":
            Camera("zed", [0, 0, 0.5], namespace="zed")
        case "franka":
            if not use_sim:
                Rviz.load_motion_planning_plugin = True
            Arm("franka", [0, 0, 0], gripper=True, moveit=True, ip_address="172.16.0.2")
        case "franka_axis":
            franka = Arm("franka", [0, 0, 0], [0, 0, 20])
            Platform("axis", [0, 0, 0.1], [0, 20, 0], parent=franka)
        case "franka_double":
            use_joystick = False
            Rviz.load_motion_planning_plugin = True
            Arm("franka", [1.0, 0, 0], gripper=True, moveit=True)
            Arm("franka", [-1.0, 0, 0], gripper=True, moveit=True)
        case "franka_realsense":
            arm = Arm("franka", [0, 0, 0], moveit=True)
            Camera("realsense", [0.05, 0, 0], [0, -90, 180], parent=arm)
        case "panther":
            Vehicle("panther", [0, 0, 0.2], namespace="panther")
        case "panther_axis":
            vehicle = Vehicle("panther", [0, 0, 0.2], orientation=[0, 0, 90])
            Platform("axis", [0, 0, 0.2], [20, 0, 0], parent=vehicle)
        case "panther_realsense":
            panther = Vehicle("panther", [0, 0, 0.2])
            Camera("realsense", [0, 0, 0.2], parent=panther)
        case "panther_gps":
            panther = Vehicle("panther", [0, 0, 0.2])
            GPS("nmea", [0, 0, 0.2], parent=panther)
        case "panther_zed":
            panther = Vehicle("panther", [0, 0, 0.2])
            Camera("zed", [0, 0, 0.5], parent=panther)
        case "panther_lidar":
            panther = Vehicle("panther", [0, 0, 0.2], navigation=True)
            Lidar("velodyne", [0.13, -0.13, 0.35], parent=panther)
        case "panther_ouster":
            panther = Vehicle("panther", [0, 0, 0.2], navigation=True)
            Lidar("ouster", [0.13, -0.13, 0.35], parent=panther)
        case "mm":
            panther = Vehicle("panther", [0, 0, 0.2])
            Arm("franka", [0, 0, 0.14], gripper=True, parent=panther, moveit=True)
        case "mm_lidar":
            panther = Vehicle("panther", [0, 0, 0.2], navigation=True)
            Arm("franka", [0, 0, 0.14], gripper=True, parent=panther, moveit=True)
            Lidar("velodyne", [0.13, -0.13, 0.35], parent=panther)
        case "mm_ouster":
            panther = Vehicle("panther", [0, 0, 0.2], navigation=True)
            Arm("franka", [0, 0, 0.14], gripper=True, parent=panther, moveit=True)
            Lidar("ouster", [0.13, -0.13, 0.35], parent=panther)
        case "panther_and_franka":
            Vehicle("panther", [0, -0.5, 0.2])
            Arm("franka", [0, 0.5, 0])

    if Platform.platforms == []:
        raise RuntimeError("No platforms specified. Please specify a platform.")
    Platform.order_platforms()

    state_publishers = Platform.create_state_publishers()
    gazebo = Platform.create_gazebo_launch(load_gazebo_ui)
    hardware_interfaces = Platform.create_hardware_interfaces()
    map_links = Platform.create_map_links()
    parent_links = Platform.create_parent_links()
    controllers = Platform.create_controllers()
    launch_descriptions = Platform.create_launch_descriptions()
    joystick_nodes = Platform.create_joystick_nodes() if use_joystick else []

    utilities = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "utils.launch.py")
    )

    Rviz.set_fixed_frame("map")
    Rviz.create_rviz_file()
    rviz = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py")
    )

    Vizanti.create_config_file()
    vizanti = RegisteredLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "vizanti.launch.py")
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        Register.group(rviz, context) if use_rviz else SKIP,
        *[Register.on_start(node, context) for node in state_publishers],
        Register.group(gazebo, context) if use_sim else SKIP,
        *[Register.group(group, context) for group in hardware_interfaces],
        *[Register.on_start(node, context) for node in map_links],
        *[Register.on_start(node, context) for node in parent_links],
        *[Register.group(node, context) for node in controllers],
        Register.group(utilities, context),
        *[Register.on_start(node, context) for node in joystick_nodes],
        *[Register.group(group, context) for group in launch_descriptions],
        Register.group(vizanti, context) if use_vizanti else SKIP,
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
            use_vizanti_arg.declaration,
            configuration_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
