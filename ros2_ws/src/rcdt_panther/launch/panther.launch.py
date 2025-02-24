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

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])
use_velodyne_arg = LaunchArgument("velodyne", False, [True, False])
use_slam_arg = LaunchArgument("slam", False, [True, False])
use_nav2_arg = LaunchArgument("nav2", False, [True, False])


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    load_gazebo_ui = load_gazebo_ui_arg.value(context)
    use_rviz = use_rviz_arg.value(context)
    use_velodyne = use_velodyne_arg.value(context)
    use_slam = use_slam_arg.value(context)
    use_nav2 = use_nav2_arg.value(context)

    if use_slam:
        use_velodyne = True

    if use_nav2:
        use_velodyne = True
        use_slam = True

    xacro_path = get_file_path("rcdt_panther", ["urdf"], "panther.urdf.xacro")
    components_path = get_file_path("rcdt_panther", ["config"], "components.yaml")
    xacro_arguments = {"use_sim": "true", "components_config_path": components_path}
    xacro_arguments["load_velodyne"] = "true" if use_velodyne else "false"
    robot_description = get_robot_description(xacro_path, xacro_arguments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="panther",
        parameters=[robot_description, {"frame_prefix": "panther/"}],
    )

    robot = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py"),
        launch_arguments={
            "world": "walls.sdf",
            "namespace": "panther",
            "velodyne": str(use_velodyne),
            "load_gazebo_ui": str(load_gazebo_ui),
        }.items(),
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace="panther",
        arguments=[
            "joint_state_broadcaster",
            "-t",
            "joint_state_broadcaster/JointStateBroadcaster",
        ],
    )

    controllers = IncludeLaunchDescription(
        get_file_path("rcdt_panther", ["launch"], "controllers.launch.py"),
    )

    if use_nav2:
        rviz_display_config = "nav2.rviz"
    elif use_slam:
        rviz_display_config = "slam.rviz"
    elif use_velodyne:
        rviz_display_config = "panther_velodyne.rviz"
    else:
        rviz_display_config = "panther_general.rviz"
    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={
            "rviz_frame": "map" if use_slam else "/panther/odom",
            "rviz_display_config": rviz_display_config,
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
        executable="joy_topic_manager.py",
    )

    joy_to_twist_panther = Node(
        package="rcdt_utilities",
        executable="joy_to_twist.py",
        parameters=[
            {"sub_topic": "/panther/joy"},
            {"pub_topic": "/panther/drive_controller/cmd_vel"},
            {"config_pkg": "rcdt_panther"},
        ],
    )

    twist_to_twist_stamped = Node(
        package="rcdt_utilities", executable="twist_to_twist_stamped.py"
    )

    slam = IncludeLaunchDescription(
        get_file_path("slam_toolbox", ["launch"], "online_async_launch.py"),
        launch_arguments={
            "slam_params_file": get_file_path(
                "rcdt_panther", ["config"], "slam_params.yaml"
            ),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        get_file_path("nav2_bringup", ["launch"], "navigation_launch.py"),
        launch_arguments={
            "params_file": get_file_path(
                "rcdt_panther", ["config"], "nav2_params.yaml"
            ),
        }.items(),
    )

    simulation_components = LaunchDescription(
        [
            robot_state_publisher,
            robot,
            joint_state_broadcaster,
            controllers,
            joy,
            joy_topic_manager,
            joy_to_twist_panther,
        ]
    )

    skip = LaunchDescriptionEntity()
    return [
        SetParameter(name="use_sim_time", value=use_sim),
        simulation_components if use_sim else skip,
        rviz if use_rviz else skip,
        slam if use_slam else skip,
        twist_to_twist_stamped if use_nav2 else skip,
        nav2 if use_nav2 else skip,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            use_rviz_arg.declaration,
            use_velodyne_arg.declaration,
            use_slam_arg.declaration,
            use_nav2_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
