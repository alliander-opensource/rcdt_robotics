# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import xml.etree.ElementTree as ET

from launch import LaunchContext, LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from rcdt_gazebo.gazebo_ros_paths import GazeboRosPaths
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path
from rcdt_utilities.register import Register

load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", False, [True, False])
world_arg = LaunchArgument("world", "walls.sdf")
platforms_arg = LaunchArgument("platforms", "")
positions_arg = LaunchArgument("positions", "")
orientations_arg = LaunchArgument("orientations", "")
parents_arg = LaunchArgument("parents", "")
parent_links_arg = LaunchArgument("parent_links", "")
bridge_topics_arg = LaunchArgument("bridge_topics", "")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Gazebo simulation with platforms.

    Args:
        context (LaunchContext): The launch context.

    Raises:
        ValueError: If the SDF file does not contain a world attribute with a name.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    load_gazebo_ui = load_gazebo_ui_arg.bool_value(context)
    world = world_arg.string_value(context)
    platforms = platforms_arg.string_value(context)
    positions = positions_arg.string_value(context)
    orientations = orientations_arg.string_value(context)
    parents = parents_arg.string_value(context)
    parent_links = parent_links_arg.string_value(context)
    bridge_topics = bridge_topics_arg.string_value(context).split()

    sdf_file = get_file_path("rcdt_gazebo", ["worlds"], world)
    sdf = ET.parse(sdf_file)
    world_attribute = sdf.getroot().find("world")
    if world_attribute is None:
        raise ValueError("sdf file should contain a world attribute with a name.")
    else:
        world_name = world_attribute.attrib.get("name")
    cmd = ["gz", "sim", sdf_file]
    if not load_gazebo_ui:
        cmd.append("-s")
    gazebo = ExecuteProcess(
        cmd=cmd,
        shell=False,
        additional_env=GazeboRosPaths.get_env(),
    )

    bridge_topics.append("/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_topics,
    )

    spawn_platforms = Node(
        package="rcdt_gazebo",
        executable="spawn_platforms.py",
        parameters=[
            {
                "platforms": platforms,
                "positions": positions,
                "orientations": orientations,
                "parents": parents,
                "parent_links": parent_links,
            },
        ],
        output="screen",
    )

    unpause_sim = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            f"/world/{world_name}/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            "pause: false",
        ],
        shell=False,
    )

    return [
        Register.on_start(gazebo, context),
        Register.on_log(
            bridge,
            "Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)]",
            context,
        ),
        Register.on_log(spawn_platforms, "All platforms spawned!", context),
        Register.on_start(unpause_sim, context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Gazebo simulation with platforms.

    Returns:
        LaunchDescription: The launch description containing the actions to be executed.
    """
    return LaunchDescription(
        [
            load_gazebo_ui_arg.declaration,
            world_arg.declaration,
            platforms_arg.declaration,
            positions_arg.declaration,
            bridge_topics_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
