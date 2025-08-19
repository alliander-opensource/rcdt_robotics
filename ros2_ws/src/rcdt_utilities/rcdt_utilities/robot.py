# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from typing import Literal

from launch_ros.actions import Node

from rcdt_utilities.launch_utils import get_file_path, get_robot_description
from rcdt_utilities.register import RegisteredLaunchDescription
from rcdt_utilities.rviz import Rviz


class Platform:
    """A class used to dynamically create all the required nodes for a platform.

    Attributes:
        platforms (list[Platform]): A list of all the platforms.
        platform_indices (dict[str, int]): A collections of the different platforms and the number of occurrences.
        names (list[str]): A list of all robot names.
        bridge_topics (list[str]): A list of all topics that should be bridged between Gazebo and ROS.
    """

    platforms: list["Platform"] = []
    platform_indices: dict[str, int] = {"panther": 0, "franka": 0, "velodyne": 0}
    names: list[str] = []
    bridge_topics: list[str] = []

    @staticmethod
    def add(platform: "Platform") -> str:
        """Add a platform instance to the general list.

        Args:
            platform (Platform): The platform instance to add.

        Returns:
            str: The namespace assigned to the platform instance.
        """
        Platform.platforms.append(platform)
        Platform.platform_indices[platform.name] += 1
        index = Platform.platform_indices[platform.name]
        return f"{platform.name}{index}"

    @staticmethod
    def create_state_publishers() -> list[Node]:
        """Create state publishers for all robots.

        Returns:
            list[Node]: A list of all state publisher nodes.
        """
        return [robot.create_state_publisher() for robot in Platform.platforms]

    @staticmethod
    def move_franka_robots_to_front() -> None:
        """Move all the Franka robots to the front of the list.

        For some reason, the ros control plugin does not load for Franka robots when another platform is loaded before.
        Therefore we can load all Franka robots before other platforms by rearranging the list using this method.
        """
        for index in range(len(Platform.platforms)):
            if Platform.platforms[index].name == "franka":
                Platform.platforms.insert(0, Platform.platforms.pop(index))

    @staticmethod
    def create_gazebo_launch(load_gazebo_ui: bool) -> RegisteredLaunchDescription:
        """Create the Gazebo launch description.

        Args:
            load_gazebo_ui (bool): Whether to load the Gazebo UI or not.

        Returns:
            RegisteredLaunchDescription: The Gazebo launch description.
        """
        Platform.move_franka_robots_to_front()
        robots = [robot.namespace for robot in Platform.platforms]
        positions = [",".join(map(str, robot.position)) for robot in Platform.platforms]

        return RegisteredLaunchDescription(
            get_file_path("rcdt_gazebo", ["launch"], "gazebo_robot.launch.py"),
            launch_arguments={
                "load_gazebo_ui": str(load_gazebo_ui),
                "robots": " ".join(robots),
                "positions": " ".join(positions),
                "bridge_topics": " ".join(Platform.bridge_topics),
            },
        )

    @staticmethod
    def create_tf_publishers() -> list[Node]:
        """Create TF publishers for all robots.

        Returns:
            list[Node]: A list of all TF publisher nodes.
        """
        tf_publishers = []
        for robot in Platform.platforms:
            if robot.parent is not None:
                tf_publishers.append(robot.create_tf_publisher())
        return tf_publishers

    @staticmethod
    def create_controllers() -> list[RegisteredLaunchDescription]:
        """Create controllers for all robots.

        Returns:
            list[RegisteredLaunchDescription]: A list of all controller launch descriptions.
        """
        controllers = []
        for robot in Platform.platforms:
            if robot.controller_path is not None:
                controllers.append(robot.create_controller())
        return controllers

    def create_world_links() -> list[Node]:
        """Create world links for all robots.

        Returns:
            list[Node]: A list of all tf nodes creating a link between the robot and world.
        """
        world_links = []
        for robot in Platform.platforms:
            if robot.parent is None:
                world_links.append(robot.create_world_link())
        return world_links

    def __init__(
        self,
        platform: Literal["panther", "franka", "velodyne"],
        position: list,
        parent: "Platform" | None = None,
    ):
        """Initialize a robot instance.

        Args:
            platform (Literal["panther", "franka", "velodyne"]): The platform type of the robot.
            position (list): The initial position of the robot.
            parent (Platform | None): The parent robot, if any.
        """
        self.name: Literal["panther", "franka", "velodyne"] = platform
        self.parent = parent
        self.childs = []
        self.namespace = Platform.add(self)
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
    def frame_prefix(self) -> str:
        """Return the frame prefix for the robot based on its namespace.

        Returns:
            str: The frame prefix for the robot.
        """
        return self.namespace + "/" if self.namespace else ""

    @property
    def controller_path(self) -> str | None:
        """Return the controller launch file path for the robot.

        Returns:
            str | None: The controller launch file path or None if not applicable.
        """
        match self.name:
            case "panther":
                package = "rcdt_panther"
            case "franka":
                package = "rcdt_franka"
            case _:
                return None

        return get_file_path(package, ["launch"], "controllers.launch.py")

    @property
    def xacro_path(self) -> str:
        """Return the xacro file path for the robot.

        Returns:
            str: The xacro file path for the robot.

        Raises:
            ValueError: If the platform is unknown.
        """
        match self.name:
            case "panther":
                return get_file_path("rcdt_panther", ["urdf"], "panther.urdf.xacro")
            case "franka":
                return get_file_path("rcdt_franka", ["urdf"], "franka.urdf.xacro")
            case "velodyne":
                return get_file_path(
                    "rcdt_sensors", ["urdf"], "rcdt_velodyne.urdf.xacro"
                )
            case _:
                raise ValueError("Unknown platform.")

    @property
    def base_link(self) -> str:
        """Return the base link for the robot based on its platform.

        Returns:
            str: The base link for the robot.

        Raises:
            ValueError: If the platform is unknown.
        """
        match self.name:
            case "panther":
                return "base_footprint"
            case "franka":
                return "fr3_link0"
            case "velodyne":
                return "base_link"
            case _:
                raise ValueError("Unknown platform.")

    def add_child(self, child: "Platform") -> None:
        """Add a child robot to this robot.

        Args:
            child (Platform): The child robot to add.
        """
        self.childs.append([child.namespace, child.base_link])

    def create_state_publisher(self) -> Node:
        """Create a state publisher node for the robot.

        Returns:
            Node: The state publisher node for the robot.
        """
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
        """Create tf publisher node for the robot.

        Returns:
            Node: The tf publisher node for the robot.
        """
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

    def create_world_link(self) -> Node:
        """Create a world link for the robot.

        Returns:
            Node: The world link node for the robot.
        """
        if self.name == "panther":
            child_frame = f"{self.namespace}/odom"
        else:
            child_frame = f"{self.namespace}/world"
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--frame-id",
                "world",
                "--child-frame-id",
                child_frame,
                "--x",
                f"{self.position[0]}",
                "--y",
                f"{self.position[1]}",
                "--z",
                f"{self.position[2]}",
            ],
        )

    def create_controller(self) -> RegisteredLaunchDescription:
        """Create a controller launch description for the robot.

        Returns:
            RegisteredLaunchDescription: The controller launch description for the robot.
        """
        return RegisteredLaunchDescription(
            self.controller_path,
            launch_arguments={"namespace": self.namespace},
        )


class Lidar(Platform):
    """Extension on Platform with lidar specific functionalities."""

    def __init__(
        self,
        platform: Literal["velodyne"],
        position: list,
        parent: Platform | None = None,
    ):
        """Initialize the Lidar platform.

        Args:
            platform (Literal["velodyne"]): The platform type.
            position (list): The position of the lidar.
            parent (Platform | None): The parent platform.
        """
        super().__init__(platform, position, parent)

        Rviz.add_point_cloud(self.namespace)
        Platform.bridge_topics.extend(
            [
                f"/{self.namespace}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                f"/{self.namespace}/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            ]
        )


class Arm(Platform):
    """Extension on Platform with arm specific functionalities."""

    def __init__(
        self,
        platform: Literal["franka"],
        position: list,
        parent: Platform | None = None,
    ):
        """Initialize the Arm platform.

        Args:
            platform (Literal["franka"]): The platform type.
            position (list): The position of the arm.
            parent (Platform | None): The parent platform.
        """
        super().__init__(platform, position, parent)


class Vehicle(Platform):
    """Extension on Platform with vehicle specific functionalities."""

    def __init__(
        self,
        platform: Literal["panther"],
        position: list,
        parent: Platform | None = None,
    ):
        """Initialize the Vehicle platform.

        Args:
            platform (Literal["panther"]): The platform type.
            position (list): The position of the vehicle.
            parent (Platform | None): The parent platform.
        """
        super().__init__(platform, position, parent)
