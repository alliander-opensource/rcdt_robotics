# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from rcdt_utilities.launch_utils import (
    LaunchArgument,
    add_prefix_in_robot_description_semantic,
    get_file_path,
    get_robot_description,
    get_yaml,
)
from rcdt_utilities.register import Register

robot_name_arg = LaunchArgument("robot_name", "")
moveit_package_name_arg = LaunchArgument("moveit_package_name", "")
servo_params_package_arg = LaunchArgument("servo_params_package", "rcdt_franka")
namespace_arg = LaunchArgument("namespace", "")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for MoveIt with Servo.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    robot_name = robot_name_arg.string_value(context)
    package_name = moveit_package_name_arg.string_value(context)
    servo_params_package = servo_params_package_arg.string_value(context)
    namespace = namespace_arg.string_value(context)

    moveit_config = MoveItConfigsBuilder(robot_name, package_name=package_name)
    moveit_config.trajectory_execution(
        get_file_path(package_name, ["config"], "moveit_controllers.yaml")
    )
    moveit_config.moveit_cpp(
        get_file_path(package_name, ["config"], "planning_pipeline.yaml")
    )
    configs = moveit_config.to_moveit_configs()

    srdf_path = get_file_path("rcdt_franka_moveit_config", ["config"], "fr3.srdf")
    robot_description_semantic = get_robot_description(srdf_path, semantic=True)
    add_prefix_in_robot_description_semantic(robot_description_semantic, namespace)

    # Parameters required for move_group:
    move_group_parameters = []
    move_group_parameters.append(robot_description_semantic)
    move_group_parameters.append({"publish_robot_description_semantic": True})
    move_group_parameters.append(configs.robot_description_kinematics)
    move_group_parameters.append(configs.joint_limits)
    move_group_parameters.append(configs.trajectory_execution)
    move_group_parameters.append(configs.planning_pipelines)
    move_group_parameters.append(configs.pilz_cartesian_limits)

    # Parameters required for moveit_manager:
    moveit_manager_parameters = []
    moveit_manager_parameters.append(configs.robot_description_kinematics)

    # Parameters required for moveit_servo:
    moveit_servo_parameters = []
    moveit_servo_parameters.append(configs.robot_description_kinematics)
    file = get_file_path(servo_params_package, ["config"], "servo_params.yaml")
    servo_config = get_yaml(file)
    if namespace:
        for param in ["joint_topic", "command_out_topic"]:
            value = servo_config[param]
            servo_config[param] = "/" + namespace + value
    moveit_servo_parameters.append({"moveit_servo": servo_config})

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=move_group_parameters,
        namespace=namespace,
    )

    moveit_manager = Node(
        package="rcdt_moveit",
        executable="moveit_manager",
        output="screen",
        parameters=moveit_manager_parameters,
        namespace=namespace,
    )

    moveit_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=moveit_servo_parameters,
        namespace=namespace,
    )

    switch_servo_type_to_twist = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            f"/{namespace}/servo_node/switch_command_type",
            "moveit_msgs/srv/ServoCommandType",
            "{command_type: 1}",
        ]
    )

    # Workaround for joystick command initialization issue (see issue #210):
    # Moving to the 'home' configuration ensures servo_node properly activates.
    move_to_home = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            f"/{namespace}/moveit_manager/move_to_configuration",
            "rcdt_messages/srv/MoveToConfiguration",
            "{configuration: home}",
        ],
        output="screen",
    )

    return [
        Register.on_log(
            move_group, "MoveGroup context initialization complete", context
        ),
        Register.on_log(
            moveit_manager, "Ready to take commands for planning group", context
        ),
        Register.on_log(moveit_servo, "Servo initialized successfully", context),
        Register.on_exit(switch_servo_type_to_twist, context),
        Register.on_exit(move_to_home, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for MoveIt with Servo.

    Returns:
        LaunchDescription: The launch description containing the actions to be executed.
    """
    return LaunchDescription(
        [
            robot_name_arg.declaration,
            moveit_package_name_arg.declaration,
            servo_params_package_arg.declaration,
            namespace_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
