# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path, get_yaml
from rcdt_utilities.register import Register

robot_name_arg = LaunchArgument("robot_name", "")
moveit_package_name_arg = LaunchArgument("moveit_package_name", "")
servo_params_package_arg = LaunchArgument("servo_params_package", "rcdt_franka")
namespace_arg = LaunchArgument("namespace", "")


def launch_setup(context: LaunchContext) -> None:
    """Setup the launch context for MoveIt with Servo.

    Args:
        context (LaunchContext): The launch context.
    """
    robot_name = robot_name_arg.value(context)
    package_name = moveit_package_name_arg.value(context)
    servo_params_package = servo_params_package_arg.value(context)
    namespace = namespace_arg.value(context)

    moveit_config = MoveItConfigsBuilder(robot_name, package_name=package_name)
    moveit_config.trajectory_execution(
        get_file_path(package_name, ["config"], "moveit_controllers.yaml")
    )
    moveit_config.moveit_cpp(
        get_file_path(package_name, ["config"], "planning_pipeline.yaml")
    )
    moveit_config = moveit_config.to_dict()

    file = get_file_path(servo_params_package, ["config"], "servo_params.yaml")
    servo_config = get_yaml(file)
    servo_params = {"moveit_servo": servo_config}
    if namespace != "":
        for param in ["joint_topic", "command_out_topic"]:
            value = servo_params["moveit_servo"][param]
            servo_params["moveit_servo"][param] = "/" + namespace + value

    moveit_manager = Node(
        package="rcdt_moveit",
        executable="moveit_manager",
        output="screen",
        parameters=[moveit_config],
        namespace=namespace,
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config],
        namespace=namespace,
    )

    moveit_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[moveit_config, servo_params],
        namespace=namespace,
    )

    return [
        Register.on_log(
            move_group, "MoveGroup context initialization complete", context
        ),
        Register.on_log(moveit_servo, "Servo initialized successfully", context),
        Register.on_log(
            moveit_manager, "Ready to take commands for planning group", context
        ),
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
