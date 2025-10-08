# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import LaunchArgument
from rcdt_utilities.register import Register

namespace_arm_arg = LaunchArgument("namespace_arm", "")
namespace_camera_arg = LaunchArgument("namespace_camera", "")


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the grasping node.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    namespace_arm = namespace_arm_arg.string_value(context)
    namespace_camera = namespace_camera_arg.string_value(context)

    graspnet_node = Node(
        package="rcdt_grasping",
        executable="generate_grasp.py",
    )

    grasping_logic = Node(
        package="rcdt_grasping",
        executable="grasp_logic.py",
        parameters=[
            {"namespace_arm": namespace_arm, "namespace_camera": namespace_camera}
        ],
    )

    return [
        Register.on_log(
            graspnet_node, "GraspNet model initialized and weights loaded", context
        ),
        Register.on_log(grasping_logic, "GraspLogicNode initialized!", context),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the grasping node.

    Returns:
        LaunchDescription: The launch description containing the grasping node.
    """
    return LaunchDescription(
        [
            namespace_arm_arg.declaration,
            namespace_camera_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
