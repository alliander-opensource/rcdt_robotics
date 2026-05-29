# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
from alliander_utilities.config_objects import Beamagine
from alliander_utilities.launch_argument import LaunchArgument
from alliander_utilities.launch_utils import static_tf_node
from alliander_utilities.register import Register, RegisteredLaunchDescription
from alliander_utilities.ros_utils import get_file_path
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

platform_arg = LaunchArgument("platform_config", "")


def launch_setup(context: LaunchContext) -> list:
    """The launch setup.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: The actions to start.
    """
    beamagine_config = Beamagine.from_str(platform_arg.string_value(context))

    l3cam_launch = RegisteredLaunchDescription(
        launch_description_source=get_file_path("l3cam_ros2", ["launch"], "l3cam_launch.xml"),
        launch_arguments={
            "stream": "true",
            "configure": "true",
            "rviz2": "false",
            "rqt_reconfigure": "false",
        },
    )

    parent = beamagine_config.parent
    static_tf = static_tf_node(
        parent_frame=f"{parent.namespace}/{parent.link}" if parent.link else "map",
        child_frame="lidar",
        position=beamagine_config.position,
        orientation=beamagine_config.orientation,
    )

    return [
        Register.on_start(static_tf, context),
        Register.group(l3cam_launch, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description.

    Returns:
        LaunchDescription: The launch description.
    """
    return LaunchDescription(
        [
            platform_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
