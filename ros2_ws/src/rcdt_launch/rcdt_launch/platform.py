# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
from typing import Literal

from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path, get_robot_description
from rcdt_utilities.register import RegisteredLaunchDescription

from rcdt_launch.environment_config import EnvironmentConfig
from rcdt_launch.rviz import Rviz


def add(platform: "Platform") -> None:
    """Add a platform instance to the general list.

    Args:
        platform (Platform): The platform instance to add.
    """
    EnvironmentConfig.platforms.append(platform)
    current_value = EnvironmentConfig.platform_indices.get(platform.platform, 0)
    EnvironmentConfig.platform_indices[platform.platform] = current_value + 1


def generate_namespace(platform: Platform) -> str:
    """Generate an unique namespace for the given platform.

    Args:
        platform (Platform): The platform.

    Returns:
        str: The unique namespace for the platform.
    """
    index = EnvironmentConfig.platform_indices.get(platform.platform, 0)
    return f"{platform.platform}{index}"


class Platform:  # noqa: PLR0904
    """A class used to dynamically create all the required nodes for a platform."""

    def __init__(  # noqa: PLR0913
        self,
        platform: Literal[
            "panther",
            "franka",
            "velodyne",
            "ouster",
            "realsense",
            "zed",
            "nmea",
            "axis",
        ],
        position: list,
        orientation: list | None = None,
        namespace: str | None = None,
        parent: "Platform" | None = None,
        parent_link: str = "",
    ):
        """Initialize a robot instance.

        Args:
            platform (Literal["panther", "franka", "velodyne", "ouster", "realsense", "zed", "nmea", "axis"]): The platform type of the robot.
            position (list): The initial position of the robot.
            orientation (list | None): The initial orientation of the robot.
            namespace (str | None): The namespace of the robot. If None, a unique namespace will be generated.
            parent (Platform | None): The parent robot, if any.
            parent_link (str): The link of the parent to which the platform is attached. If empty, the base_link of the parent is used.
        """
        self.platform = platform
        self.parent = parent
        self.childs = []
        add(self)
        self.namespace = namespace if namespace else generate_namespace(self)

        Rviz.add_robot_model(self.namespace)

        self.position = position
        self.orientation = (
            list(map(math.radians, orientation)) if orientation else [0, 0, 0]
        )

        if parent is None:
            self.is_child = False
            self.parent_link = "none"
        else:
            self.is_child = True
            self.parent_link = (
                parent_link if parent_link else parent.default_connect_link
            )
            parent.add_child(self)

    @property
    def robot_description(self) -> dict:
        """Return the robot description for the robot.

        Returns:
            dict: The robot description for the robot.
        """
        xacro_arguments = {
            "simulation": str(EnvironmentConfig.simulation),
            "namespace": self.namespace,
        }
        xacro_arguments["childs"] = str(self.childs)
        xacro_arguments["parent"] = "" if self.is_child else "world"

        return get_robot_description(self.xacro_path, xacro_arguments)

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
        match self.platform:
            case "panther":
                package = "rcdt_panther"
            case "franka":
                package = "rcdt_franka"
            case _:
                return None

        return get_file_path(package, ["launch"], "controllers.launch.py")

    @property
    def xacro_path(self) -> str:  # noqa: PLR0911
        """Return the xacro file path for the robot.

        Returns:
            str: The xacro file path for the robot.

        Raises:
            ValueError: If the platform is unknown.
        """
        match self.platform:
            case "panther":
                return get_file_path("rcdt_panther", ["urdf"], "panther.urdf.xacro")
            case "franka":
                return get_file_path("rcdt_franka", ["urdf"], "franka.urdf.xacro")
            case "velodyne":
                return get_file_path(
                    "rcdt_sensors", ["urdf"], "rcdt_velodyne.urdf.xacro"
                )
            case "ouster":
                return get_file_path(
                    "rcdt_sensors", ["urdf"], "rcdt_os1_128.urdf.xacro"
                )
            case "realsense":
                return get_file_path(
                    "rcdt_sensors", ["urdf"], "rcdt_realsense_d435.urdf.xacro"
                )
            case "zed":
                return get_file_path("rcdt_sensors", ["urdf"], "rcdt_zed2i.urdf.xacro")
            case "nmea":
                return get_file_path(
                    "rcdt_sensors", ["urdf"], "rcdt_nmea_navsat.urdf.xacro"
                )
            case "axis":
                return get_file_path("rcdt_sensors", ["urdf"], "rcdt_axis.urdf.xacro")
            case _:
                raise ValueError("Cannot provide xacro path: unknown platform.")

    @property
    def base_link(self) -> str:  # noqa: PLR0911
        """Return the base link for the robot based on its platform.

        Returns:
            str: The base link for the robot.

        Raises:
            ValueError: If the platform is unknown.
        """
        match self.platform:
            case "panther":
                return "base_link"
            case "franka":
                return "fr3_link0"
            case "velodyne":
                return "base_link"
            case "ouster":
                return "base_link"
            case "realsense":
                return "base_link"
            case "zed":
                return "base_link"
            case "nmea":
                return "base_link"
            case "axis":
                return "base_link"
            case _:
                raise ValueError("Unable to provide base_link: Unknown platform.")

    @property
    def default_connect_link(self) -> str:
        """Return the default link to which child platforms should connect.

        Returns:
            str: The default connection link for the platform.

        Raises:
            ValueError: If the platform is unknown.
        """
        match self.platform:
            case "panther":
                return "base_link"
            case "franka":
                return "fr3_hand"
            case _:
                raise ValueError(
                    "Unable to provide default connection link: Unknown platform."
                )

    def add_child(self, child: "Platform") -> None:
        """Add a child robot to this robot.

        Args:
            child (Platform): The child robot to add.
        """
        self.childs.append([child.parent_link, child.namespace, child.base_link])

    def create_state_publisher(self) -> Node | None:
        """Create a state publisher node for the robot.

        Returns:
            Node | None: The state publisher node for the robot or None if not applicable.
        """
        if not EnvironmentConfig.simulation and self.platform == "panther":
            return None

        return Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=self.namespace,
            parameters=[
                self.robot_description,
                {"frame_prefix": self.frame_prefix},
                {"publish_frequency": 1000.0},
            ],
        )

    def create_parent_link(self) -> Node:
        """Create a static_transform_publisher node that links the platform to its parent.

        Returns:
            Node: The static_transform_publisher node.
        """
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_tf_{self.parent.namespace}_to_{self.namespace}",
            arguments=[
                "--frame-id",
                f"/{self.parent.namespace}/{self.parent_link}",
                "--child-frame-id",
                f"/{self.namespace}/{self.base_link}",
                "--x",
                f"{self.position[0]}",
                "--y",
                f"{self.position[1]}",
                "--z",
                f"{self.position[2]}",
                "--roll",
                f"{self.orientation[0]}",
                "--pitch",
                f"{self.orientation[1]}",
                "--yaw",
                f"{self.orientation[2]}",
            ],
        )

    def joystick_nodes(self) -> list[Node]:  # noqa: PLR6301
        """Return the required nodes to control the platform with a joystick.

        Returns:
            list[Node]: The joystick nodes for the platform.
        """
        return []

    def create_map_link(self) -> Node | None:  # TODO: naar classen zelf opsplitsen!
        """Create a static_transform_publisher node that links the platform to the map.

        If the platform is a Vehicle that is using SLAM or navigation, None is returned.
        In this case, a localization node will make the link to the 'map' frame.
        If a vehicle is not using SLAM or navigation, the 'odom' frame is directly linked to the 'map' frame.

        Returns:
            Node | None: A static_transform_publisher node that links the platform with the world or None if not applicable.
        """
        if self.platform == "panther":
            if self.navigation or self.slam:
                return None
            child_frame = f"{self.namespace}/odom"
        if self.platform == "franka" and not EnvironmentConfig.simulation:  # elif
            child_frame = f"{self.namespace}/base"
        else:
            child_frame = f"{self.namespace}/world"

        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_world",
            arguments=[
                "--frame-id",
                "map",
                "--child-frame-id",
                child_frame,
                "--x",
                f"{self.position[0]}",
                "--y",
                f"{self.position[1]}",
                "--z",
                f"{self.position[2]}",
                "--roll",
                f"{self.orientation[0]}",
                "--pitch",
                f"{self.orientation[1]}",
                "--yaw",
                f"{self.orientation[2]}",
            ],
        )

    def create_controller(self) -> RegisteredLaunchDescription:
        """Create a controller launch description for the robot.

        Returns:
            RegisteredLaunchDescription: The controller launch description for the robot.
        """
        launch_arguments = {
            "simulation": str(EnvironmentConfig.simulation),
            "namespace": self.namespace,
        }
        if self.platform == "franka":  # isinstance(self, Arm) and
            launch_arguments["ip_address"] = self.ip_address
        return RegisteredLaunchDescription(
            self.controller_path, launch_arguments=launch_arguments
        )

    def create_launch_description(self) -> list[RegisteredLaunchDescription]:  # noqa: PLR6301
        """Create the launch description for the specific platform.

        Returns:
            list[RegisteredLaunchDescription]: The launch description for the platform.
        """
        return []
