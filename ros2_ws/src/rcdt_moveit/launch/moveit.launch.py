from launch import LaunchDescription
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path, get_moveit_parameters, get_yaml


def generate_launch_description() -> LaunchDescription:
    moveit_config = get_moveit_parameters(
        robot_name="fr3",
        package_name="rcdt_franka_moveit_config",
        mode="node",
    )

    file = get_file_path("rcdt_franka", ["config"], "servo_params.yaml")
    servo_config = get_yaml(file)
    servo_params = {"moveit_servo": servo_config}

    moveit_client = Node(
        package="rcdt_moveit",
        executable="moveit_client",
        output="screen",
        parameters=[moveit_config, servo_params],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config],
    )

    return LaunchDescription(
        [
            move_group,
            moveit_client,
        ]
    )
