# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import yaml
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import (
    SKIP,
    LaunchArgument,
    get_file_path,
    get_robot_description,
    get_yaml,
)
from rcdt_utilities.register import Register, RegisteredLaunchDescription

use_sim_arg = LaunchArgument("simulation", True, [True, False])
load_gazebo_ui_arg = LaunchArgument("load_gazebo_ui", True, [True, False])
use_rviz_arg = LaunchArgument("rviz", True, [True, False])

robots = []
positions = []

controllers = []
state_publishers = []
tf_publishers = []

rviz_yaml = get_yaml(get_file_path("rcdt_utilities", ["rviz"], "default.rviz"))
rviz_yaml["Visualization Manager"]["Global Options"]["Fixed Frame"] = (
    "panther1/base_link"
)


class Robot:
    def __init__(
        self,
        namespace: str,
        position: list,
        xacro_path: str,
    ):
        self.namespace = namespace
        self.position = position
        self.xacro_path = xacro_path
        self.childs = []
        self.child_links = []

    def add_child(
        self,
        namespace: str,
        parent_link: str,
        child_link: str,
        relative_position: list,
        xacro_path: str,
    ):
        position = [sum(x) for x in zip(self.position, relative_position, strict=False)]
        self.childs.append(Robot(namespace, position, xacro_path))
        self.child_links.append([namespace, child_link])

        tf_publishers.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--frame-id",
                    f"/{self.namespace}/{parent_link}",
                    "--child-frame-id",
                    f"/{namespace}/{child_link}",
                    "--x",
                    f"{position[0]}",
                    "--y",
                    f"{position[1]}",
                    "--z",
                    f"{position[2]}",
                ],
            )
        )

    def create(self, is_child: bool = False) -> None:
        if "franka" in self.namespace:
            robots.insert(0, self.namespace)
            positions.insert(0, ",".join(str(axis) for axis in self.position))
        else:
            robots.append(self.namespace)
            positions.append(",".join(str(axis) for axis in self.position))
        self.add_to_rviz()

        frame_prefix = self.namespace + "/" if self.namespace else ""
        xacro_arguments = {"simulation": "true", "namespace": self.namespace}
        xacro_arguments["childs"] = str(self.child_links)
        xacro_arguments["parent"] = "" if is_child else "world"
        robot_description = get_robot_description(self.xacro_path, xacro_arguments)

        state_publishers.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=self.namespace,
                parameters=[robot_description, {"frame_prefix": frame_prefix}],
            )
        )

        if "franka" in self.namespace:
            controllers.append(
                RegisteredLaunchDescription(
                    get_file_path("rcdt_franka", ["launch"], "controllers.launch.py"),
                    launch_arguments={"namespace": self.namespace},
                )
            )
        elif "panther" in self.namespace:
            controllers.append(
                RegisteredLaunchDescription(
                    get_file_path("rcdt_panther", ["launch"], "controllers.launch.py"),
                    launch_arguments={"namespace": self.namespace},
                )
            )

        for child in self.childs:
            child.create(is_child=True)

    def add_to_rviz(self):
        displays: list = rviz_yaml["Visualization Manager"]["Displays"]
        displays.append(
            {
                "Alpha": "1",
                "Class": "rviz_default_plugins/RobotModel",
                "Enabled": "true",
                "Description Topic": {"Value": f"/{self.namespace}/robot_description"},
                "TF Prefix": self.namespace,
                "Name": self.namespace,
            }
        )


def launch_setup(context: LaunchContext) -> list:
    """Setup the launch description for the Panther robot.

    Args:
        context (LaunchContext): The launch context.

    Returns:
        list: A list of actions to be executed in the launch description.
    """
    use_sim = use_sim_arg.bool_value(context)
    load_gazebo_ui = load_gazebo_ui_arg.bool_value(context)
    use_rviz = use_rviz_arg.bool_value(context)

    panther_xacro_path = get_file_path("rcdt_panther", ["urdf"], "panther.urdf.xacro")
    franka_xacro_path = get_file_path("rcdt_franka", ["urdf"], "franka.urdf.xacro")
    velodyne_xacro_path = get_file_path(
        "rcdt_sensors", ["urdf"], "rcdt_velodyne.urdf.xacro"
    )

    panther1 = Robot("panther1", [0, 0, 0.2], panther_xacro_path)
    panther1.add_child(
        "franka1",
        "base_footprint",
        "fr3_link0",
        [0, 0, 0.14],
        franka_xacro_path,
    )
    panther1.add_child(
        "velodyne1",
        "base_footprint",
        "base_link",
        [0.13, -0.13, 0.35],
        velodyne_xacro_path,
    )
    panther1.create()

    Robot("franka2", [-1, 1, 0], franka_xacro_path).create()
    Robot("panther2", [-1, 0, 0.2], panther_xacro_path).create()
    Robot("velodyne2", [-1, -1, 0], velodyne_xacro_path).create()

    gazebo = RegisteredLaunchDescription(
        get_file_path("rcdt_gazebo", ["launch"], "gazebo_robot.launch.py"),
        launch_arguments={
            "load_gazebo_ui": str(load_gazebo_ui),
            "robots": " ".join(robots),
            "positions": " ".join(positions),
        },
    )

    with open("/tmp/rviz.rviz", "w", encoding="utf-8") as outfile:
        yaml.dump(rviz_yaml, outfile, default_flow_style=False)

    rviz = RegisteredLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py")
    )

    return [
        SetParameter(name="use_sim_time", value=use_sim),
        *[Register.on_start(publisher, context) for publisher in state_publishers],
        Register.group(gazebo, context) if use_sim else SKIP,
        *[Register.on_start(tf_publisher, context) for tf_publisher in tf_publishers],
        *[Register.group(controller, context) for controller in controllers],
        Register.group(rviz, context) if use_rviz else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the Panther robot.

    Returns:
        LaunchDescription: The launch description for the Panther robot.
    """
    return LaunchDescription(
        [
            use_sim_arg.declaration,
            load_gazebo_ui_arg.declaration,
            use_rviz_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
