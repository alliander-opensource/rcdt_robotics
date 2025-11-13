# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from typing import Literal

from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path, get_robot_description
from rcdt_utilities.register import RegisteredLaunchDescription

from rcdt_launch.camera import Camera
from rcdt_launch.environment_config import EnvironmentConfig
from rcdt_launch.moveit import Moveit
from rcdt_launch.platform import Platform
from rcdt_launch.rviz import Rviz


class Arm(Platform):
    """Extension on Platform with arm specific functionalities."""

    def __init__(  # noqa: PLR0913
        self,
        platform: Literal["franka"],
        position: list,
        orientation: list | None = None,
        namespace: str | None = None,
        parent: Platform | None = None,
        parent_link: str = "",
        moveit: bool = False,
        gripper: bool = False,
        ip_address: str = "",
    ):
        """Initialize the Arm platform.

        Args:
            platform (Literal["franka"]): The platform type.
            position (list): The position of the arm.
            orientation (list | None): The initial orientation of the arm.
            namespace (str | None): The namespace of the arm.
            parent (Platform | None): The parent platform.
            parent_link (str): The link of the parent to which the platform is attached. If empty, the base_link of the parent is used.
            moveit (bool): Whether to use MoveIt for the arm.
            gripper (bool): Whether to add a start the gripper services.
            ip_address (str): The IP address of the arm.
        """
        super().__init__(
            platform, position, orientation, namespace, parent, parent_link
        )
        self.platform = platform
        self.moveit = moveit
        self.gripper = gripper
        self.ip_address = ip_address

        self.camera: Camera | None = None

        if moveit:
            Moveit.add(self.namespace, self.robot_description, self.platform)
            Rviz.moveit_namespaces.append(self.namespace)
            Rviz.add_motion_planning_plugin(self.namespace)
            Rviz.add_planning_scene(self.namespace)
            Rviz.add_robot_state(self.namespace)
            Rviz.add_trajectory(self.namespace)

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
        if self.platform == "franka":
            xacro_arguments["ip_address"] = self.ip_address

        return get_robot_description(self.xacro_path, xacro_arguments)

    def create_launch_description(self) -> list[RegisteredLaunchDescription]:
        """Create the launch description with specific elements for an arm.

        Returns:
            list[RegisteredLaunchDescription]: The launch description for the platform.
        """
        launch_descriptions = []
        if self.gripper:
            launch_descriptions.append(self.create_gripper_launch())
        if self.moveit:
            launch_descriptions.append(self.create_moveit_launch())
        return launch_descriptions

    def joystick_nodes(self) -> list[Node]:
        """Return the required nodes to control the platform with a joystick.

        Returns:
            list[Node]: The joystick nodes for the platform.
        """
        nodes: list[Node] = []

        if self.platform == "franka":
            nodes.append(
                Node(
                    package="rcdt_joystick",
                    executable="joy_to_twist.py",
                    parameters=[
                        {"sub_topic": f"/{self.namespace}/joy"},
                        {"pub_topic": f"/{self.namespace}/servo_node/delta_twist_cmds"},
                        {"config_pkg": "rcdt_franka"},
                        {"pub_frame": f"{self.namespace}/fr3_hand"},
                    ],
                    namespace=self.namespace,
                )
            )
            nodes.append(
                Node(
                    package="rcdt_joystick",
                    executable="joy_to_gripper.py",
                    parameters=[
                        {"config_pkg": "rcdt_franka"},
                    ],
                    namespace=self.namespace,
                )
            )

        return nodes

    def create_gripper_launch(self) -> RegisteredLaunchDescription | None:
        """Create the gripper services launch description.

        Returns:
            RegisteredLaunchDescription | None: The launch description for the gripper services.
        """
        if self.platform == "franka":
            return RegisteredLaunchDescription(
                get_file_path("rcdt_franka", ["launch"], "gripper_services.launch.py"),
                launch_arguments={"namespace": self.namespace},
            )

        return None

    def create_moveit_launch(self) -> RegisteredLaunchDescription:
        """Create the MoveIt launch description.

        Returns:
            RegisteredLaunchDescription: The launch description for the MoveIt.
        """
        return RegisteredLaunchDescription(
            get_file_path("rcdt_moveit", ["launch"], "moveit.launch.py"),
            launch_arguments={
                "namespace_arm": self.namespace,
                "namespace_camera": self.camera.namespace if self.camera else "",
            },
        )
