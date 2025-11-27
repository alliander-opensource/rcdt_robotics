# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node
from rcdt_utilities.ros_utils import get_file_path, get_robot_description

namespace = "panther"

xacro_arguments = {
    "simulation": str(True),
    "namespace": namespace,
}
xacro_arguments["childs"] = str([])
xacro_arguments["parent"] = "world"

xacro_path = get_file_path("rcdt_husarion", ["urdf"], "panther.urdf.xacro")
robot_description = get_robot_description(xacro_path, xacro_arguments)

frame_prefix = f"{namespace}/"


robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    namespace=namespace,
    parameters=[
        robot_description,
        {"frame_prefix": frame_prefix},
        {"publish_frequency": 1000.0},
    ],
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([robot_state_publisher])
