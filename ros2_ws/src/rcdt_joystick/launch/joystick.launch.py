# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import SKIP, LaunchArgument
from rcdt_utilities.register import Register

use_sim_arg = LaunchArgument("simulation", True, [True, False])
robots_arg = LaunchArgument("robots", "")
use_collision_monitor_arg = LaunchArgument("collision_monitor", False, [True, False])
scale_speed_arg = LaunchArgument(
    "scale_speed", default_value=0.4, min_value=0.0, max_value=1.0
)


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the joystick nodes.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)
    robots = robots_arg.string_value(context).split(" ")
    use_collision_monitor = use_collision_monitor_arg.bool_value(context)
    scale_speed = scale_speed_arg.float_value(context)

    joy = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[
            {"sticky_buttons": True},
        ],
    )

    joy_topic_manager = Node(
        package="rcdt_joystick",
        executable="joy_topic_manager.py",
    )

    joy_to_gripper_franka = Node(
        package="rcdt_joystick",
        executable="joy_to_gripper.py",
        parameters=[
            {"config_pkg": "rcdt_franka"},
        ],
        namespace="franka",
    )

    joy_to_twist_franka = Node(
        package="rcdt_joystick",
        executable="joy_to_twist.py",
        parameters=[
            {"sub_topic": "/franka/joy"},
            {"pub_topic": "/franka/servo_node/delta_twist_cmds"},
            {"config_pkg": "rcdt_franka"},
            {"pub_frame": "fr3_hand"},
        ],
        namespace="franka",
    )

    pub_topic = (
        "/panther/cmd_vel" if not use_collision_monitor else "/panther/cmd_vel_raw"
    )

    joy_to_twist_panther = Node(
        package="rcdt_joystick",
        executable="joy_to_twist.py",
        parameters=[
            {"sub_topic": "/panther/joy"},
            {"pub_topic": pub_topic},
            {"config_pkg": "rcdt_panther"},
            {"scale": 1.0 if use_sim else scale_speed},
        ],
        namespace="panther",
    )

    return [
        Register.on_start(joy, context) if robots != [""] else SKIP,
        Register.on_start(joy_topic_manager, context) if robots != [""] else SKIP,
        Register.on_start(joy_to_gripper_franka, context)
        if "franka" in robots
        else SKIP,
        Register.on_start(joy_to_twist_franka, context) if "franka" in robots else SKIP,
        Register.on_start(joy_to_twist_panther, context)
        if "panther" in robots
        else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the joystick nodes.

    Returns:
        LaunchDescription: The launch description containing the joystick nodes.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            robots_arg.declaration,
            use_collision_monitor_arg.declaration,
            scale_speed_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
