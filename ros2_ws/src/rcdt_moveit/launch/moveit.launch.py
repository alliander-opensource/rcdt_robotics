# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from rcdt_launch.moveit import Moveit
from rcdt_utilities.launch_utils import LaunchArgument
from rcdt_utilities.register import Register

namespace_arg = LaunchArgument("namespace", "")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for MoveIt with Servo.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    namespace = namespace_arg.string_value(context)
    configuration = Moveit.configurations[namespace]

    # Parameters required for move_group:
    move_group_parameters = []
    move_group_parameters.append(configuration.robot_description)
    move_group_parameters.append(configuration.robot_description_semantic)
    move_group_parameters.append(configuration.robot_description_kinematics)
    move_group_parameters.append(configuration.joint_limits)
    move_group_parameters.append(configuration.trajectory_execution)
    move_group_parameters.append(configuration.planning_pipelines)
    move_group_parameters.append(configuration.pilz_cartesian_limits)

    # Parameters required for moveit_manager:
    moveit_manager_parameters = []
    moveit_manager_parameters.append(configuration.robot_description)
    moveit_manager_parameters.append(configuration.robot_description_semantic)
    moveit_manager_parameters.append(configuration.robot_description_kinematics)

    # Parameters required for moveit_servo:
    moveit_servo_parameters = []
    moveit_servo_parameters.append(configuration.robot_description)
    moveit_servo_parameters.append(configuration.robot_description_semantic)
    moveit_servo_parameters.append(configuration.robot_description_kinematics)
    moveit_servo_parameters.append(Moveit.servo_configurations[namespace])

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
            namespace_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
