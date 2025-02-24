# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import (
    LaunchArgument,
    get_file_path,
    get_robot_description,
)

load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
use_velodyne_arg = LaunchArgument("velodyne", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    use_rviz = use_rviz_arg.value(context)
    use_velodyne = use_velodyne_arg.value(context)

    xacro_path = get_file_path("rcdt_panther", ["urdf"], "panther.urdf.xacro")
    components_path = get_file_path("rcdt_panther", ["config"], "components.yaml")
    xacro_arguments = {"use_sim": "true", "components_config_path": components_path}
    xacro_arguments["load_velodyne"] = "true" if use_velodyne else "false"
    robot_description = get_robot_description(xacro_path, xacro_arguments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    robot = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py"),
        launch_arguments={
            "world": "walls.sdf",
            "velodyne": str(use_velodyne),
            "load_gazebo_ui": str(load_gazebo_ui),
        }.items(),
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

    controllers = IncludeLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "controllers.launch.py"),
    )

    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={
            "rviz_display_config": "lidar.rviz" if use_velodyne else "general.rviz",
        }.items(),
    )

    collision_monitoring = IncludeLaunchDescription(
        get_file_path("nav2_collision_monitor", ["launch"], "collision_monitor_node.launch.py"),
        launch_arguments={
            "params_file": "/home/rcdt/rcdt_robotics/ros2_ws/src/rcdt_sensors/config/collision_monitor_params.yaml",
        }.items(),
    )

    twist_to_twist_stamped = Node(
        package="rcdt_utilities",
        executable="twist_to_twist_stamped.py",
        parameters=[
            {"sub_topic": "/coll_mntr_cmd_vel"}, 
            {"pub_topic": "/diff_drive_controller/cmd_vel"}
            ],
    )

    joy = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[
            {"sticky_buttons": True},
        ],
    )

    joy_to_twist = Node(
        package="rcdt_utilities",
        executable="joy_to_twist.py",
        parameters=[
            {"pub_topic": "/joy_cmd_vel"},
            {"config_pkg": "rcdt_panther"},
        ],
    )

    skip = LaunchDescriptionEntity()
    return [
        SetParameter(name="use_sim_time", value=True),
        robot_state_publisher,
        robot,
        static_transform_publisher,
        joint_state_broadcaster,
        controllers,
        rviz if use_rviz else skip,
        collision_monitoring,
        twist_to_twist_stamped,
        joy,
        joy_to_twist,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            load_gazebo_ui_arg.declaration,
            use_rviz_arg.declaration,
            use_velodyne_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
