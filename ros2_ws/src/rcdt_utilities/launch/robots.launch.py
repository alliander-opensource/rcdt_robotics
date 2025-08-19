# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from typing import Literal

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


class Rviz:
    yaml = get_yaml(get_file_path("rcdt_utilities", ["rviz"], "default.rviz"))

    @staticmethod
    def set_fixed_frame(frame: str) -> None:
        Rviz.yaml["Visualization Manager"]["Global Options"]["Fixed Frame"] = frame

    @staticmethod
    def add_robot_model(namespace: str) -> None:
        print(f"Add robot model for namespace: {namespace}")
        displays: list = Rviz.yaml["Visualization Manager"]["Displays"]
        displays.append(
            {
                "Alpha": "1",
                "Class": "rviz_default_plugins/RobotModel",
                "Enabled": "true",
                "Description Topic": {"Value": f"/{namespace}/robot_description"},
                "TF Prefix": namespace,
                "Name": namespace,
            }
        )

    @staticmethod
    def create_rviz_file() -> None:
        print("Create rviz file")
        with open("/tmp/rviz.rviz", "w", encoding="utf-8") as outfile:
            yaml.dump(Rviz.yaml, outfile, default_flow_style=False)


class Robot:
    robots: list["Robot"] = []
    platforms = {"panther": 0, "franka": 0, "velodyne": 0}
    names: list[str] = []

    @staticmethod
    def add(robot: "Robot") -> str:
        Robot.robots.append(robot)
        Robot.platforms[robot.platform] += 1
        index = Robot.platforms[robot.platform]
        return f"{robot.platform}{index}"

    @staticmethod
    def create_state_publishers() -> list[Node]:
        return [robot.create_state_publisher() for robot in Robot.robots]

    @staticmethod
    def move_franka_robots_to_front() -> None:
        for index in range(len(Robot.robots)):
            if Robot.robots[index].platform == "franka":
                Robot.robots.insert(0, Robot.robots.pop(index))

    @staticmethod
    def create_gazebo_launch(load_gazebo_ui: bool) -> RegisteredLaunchDescription:
        Robot.move_franka_robots_to_front()
        robots = [robot.namespace for robot in Robot.robots]
        positions = [",".join(map(str, robot.position)) for robot in Robot.robots]

        return RegisteredLaunchDescription(
            get_file_path("rcdt_gazebo", ["launch"], "gazebo_robot.launch.py"),
            launch_arguments={
                "load_gazebo_ui": str(load_gazebo_ui),
                "robots": " ".join(robots),
                "positions": " ".join(positions),
            },
        )

    @staticmethod
    def create_tf_publishers() -> list[Node]:
        tf_publishers = []
        for robot in Robot.robots:
            if robot.parent is not None:
                tf_publishers.append(robot.create_tf_publisher())
        return tf_publishers

    @staticmethod
    def create_controllers() -> list[RegisteredLaunchDescription]:
        controllers = []
        for robot in Robot.robots:
            if robot.controller_path is not None:
                controllers.append(robot.create_controller())
        return controllers

    def __init__(
        self,
        platform: Literal["panther", "franka", "velodyne"],
        position: list,
        parent: "Robot" | None = None,
    ):
        self.platform = platform
        self.parent = parent
        self.childs = []
        self.namespace = Robot.add(self)
        Rviz.add_robot_model(self.namespace)

        if parent is None:
            self.is_child = False
            self.position = position
        else:
            self.is_child = True
            self.position = [
                sum(x) for x in zip(parent.position, position, strict=False)
            ]
            parent.add_child(self)

    @property
    def frame_prefix(self):
        return self.namespace + "/" if self.namespace else ""

    @property
    def controller_path(self):
        match self.platform:
            case "panther":
                package = "rcdt_panther"
            case "franka":
                package = "rcdt_franka"
            case _:
                return None

        return get_file_path(package, ["launch"], "controllers.launch.py")

    @property
    def xacro_path(self):
        match self.platform:
            case "panther":
                return get_file_path("rcdt_panther", ["urdf"], "panther.urdf.xacro")
            case "franka":
                return get_file_path("rcdt_franka", ["urdf"], "franka.urdf.xacro")
            case "velodyne":
                return get_file_path(
                    "rcdt_sensors", ["urdf"], "rcdt_velodyne.urdf.xacro"
                )

    @property
    def base_link(self):
        match self.platform:
            case "panther":
                return "base_footprint"
            case "franka":
                return "fr3_link0"
            case "velodyne":
                return "base_link"

    def add_child(self, child: "Robot") -> None:
        self.childs.append([child.namespace, child.base_link])

    def create_state_publisher(self) -> Node:
        xacro_arguments = {"simulation": "true", "namespace": self.namespace}
        xacro_arguments["childs"] = str(self.childs)
        xacro_arguments["parent"] = "" if self.is_child else "world"
        robot_description = get_robot_description(self.xacro_path, xacro_arguments)

        return Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=self.namespace,
            parameters=[robot_description, {"frame_prefix": self.frame_prefix}],
        )

    def create_tf_publisher(self) -> Node:
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--frame-id",
                f"/{self.parent.namespace}/{self.parent.base_link}",
                "--child-frame-id",
                f"/{self.namespace}/{self.base_link}",
                "--x",
                f"{self.position[0]}",
                "--y",
                f"{self.position[1]}",
                "--z",
                f"{self.position[2]}",
            ],
        )

    def create_controller(self) -> RegisteredLaunchDescription:
        return RegisteredLaunchDescription(
            self.controller_path,
            launch_arguments={"namespace": self.namespace},
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

    panther1 = Robot("panther", [0, 0, 0.2])
    Robot("franka", [0, 0, 0.14], parent=panther1)
    Robot("velodyne", [0.13, -0.13, 0.35], parent=panther1)

    state_publishers = Robot.create_state_publishers()
    gazebo = Robot.create_gazebo_launch(load_gazebo_ui)
    tf_publishers = Robot.create_tf_publishers()
    controllers = Robot.create_controllers()

    Rviz.set_fixed_frame(f"{panther1.namespace}/base_link")
    Rviz.create_rviz_file()
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
