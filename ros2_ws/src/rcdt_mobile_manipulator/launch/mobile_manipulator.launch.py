# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter

from rcdt_utilities.launch_utils import (
    get_file_path,
    get_robot_description,
    LaunchArgument,
)

moveit_mode_arg = LaunchArgument("moveit", "off", ["node", "rviz", "servo", "off"])
use_rviz_arg = LaunchArgument("rviz", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    moveit_mode = moveit_mode_arg.value(context)
    use_rviz = use_rviz_arg.value(context)

    xacro_file = get_file_path(
        "rcdt_mobile_manipulator", ["urdf"], "mobile_manipulator.urdf.xacro"
    )
    robot_description = get_robot_description(xacro_file)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    robot = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py")
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world",
        arguments=["--frame-id", "world", "--child-frame-id", "odom"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-t",
            "joint_state_broadcaster/JointStateBroadcaster",
        ],
    )

    franka_controllers = IncludeLaunchDescription(
        get_file_path("rcdt_franka", ["launch"], "controllers.launch.py"),
        launch_arguments={
            "simulation": "True",
            "arm_controller": "fr3_arm_controller",
            "gripper_controller": "fr3_gripper",
        }.items(),
    )

    panther_controllers = IncludeLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "controllers.launch.py")
    )

    rviz_display_config = get_file_path("rcdt_utilities", ["rviz"], "markers.rviz")
    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={"rviz_display_config": rviz_display_config}.items(),
    )
    load_rviz = use_rviz and moveit_mode != "rviz"

    moveit = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "moveit.launch.py"),
        launch_arguments={
            "moveit_config_package": "rcdt_mobile_manipulator_moveit_config"
        }.items(),
    )

    joy = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[
            {"sticky_buttons": True},
        ],
    )

    joy_topic_manager = Node(
        package="rcdt_mobile_manipulator",
        executable="joy_topic_manager_node.py",
    )

    joy_to_twist_franka = Node(
        package="rcdt_utilities",
        executable="joy_to_twist_node.py",
        namespace="franka",
        parameters=[
            {"sub_topic": "/franka/joy"},
            {"pub_topic": "/servo_node/delta_twist_cmds"},
            {"config_pkg": "rcdt_franka"},
            {"pub_frame": "fr3_hand"},
        ],
    )

    joy_to_twist_panther = Node(
        package="rcdt_utilities",
        executable="joy_to_twist_node.py",
        namespace="panther",
        parameters=[
            {"sub_topic": "/panther/joy"},
            {"pub_topic": "/diff_drive_controller/cmd_vel"},
            {"config_pkg": "rcdt_panther"},
        ],
    )

    joy_to_gripper = Node(
        package="rcdt_franka",
        executable="joy_to_gripper_node.py",
        parameters=[{"sub_topic": "/franka/joy"}],
    )

    skip = LaunchDescriptionEntity()
    return [
        SetParameter(name="use_sim_time", value=True),
        robot_state_publisher,
        robot,
        static_transform_publisher,
        joint_state_broadcaster,
        franka_controllers,
        panther_controllers,
        rviz if load_rviz else skip,
        moveit if moveit_mode != "off" else skip,
        joy,
        joy_topic_manager,
        joy_to_twist_franka if moveit_mode == "servo" else skip,
        joy_to_twist_panther,
        joy_to_gripper,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_rviz_arg.declaration,
            moveit_mode_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
