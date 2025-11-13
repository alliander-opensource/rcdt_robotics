# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from typing import Literal

from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path
from rcdt_utilities.register import RegisteredLaunchDescription

from rcdt_launch.camera import Camera
from rcdt_launch.environment_config import EnvironmentConfig
from rcdt_launch.lidar import Lidar
from rcdt_launch.platform import Platform
from rcdt_launch.rviz import Rviz
from rcdt_launch.vizanti import Vizanti


class Vehicle(Platform):
    """Extension on Platform with vehicle specific functionalities."""

    def __init__(  # noqa: PLR0913
        self,
        platform: Literal["panther"],
        position: list,
        orientation: list | None = None,
        namespace: str | None = None,
        parent: Platform | None = None,
        parent_link: str = "",
        navigation: bool = False,
        slam: bool = False,
        collision_monitor: bool = False,
    ):
        """Initialize the Vehicle platform.

        Args:
            platform (Literal["panther"]): The platform type.
            position (list): The position of the vehicle.
            orientation (list | None): The initial orientation of the vehicle.
            namespace (str | None): The namespace of the vehicle.
            parent (Platform | None): The parent platform.
            parent_link (str): The link of the parent to which the platform is attached. If empty, the base_link of the parent is used.
            navigation (bool): Whether to start navigation for the vehicle.
            slam (bool): Whether to start SLAM for the vehicle.
            collision_monitor (bool): Whether to start the collision monitor for the vehicle.
        """
        super().__init__(
            platform, position, orientation, namespace, parent, parent_link
        )
        self.platform = platform
        self.namespace = self.namespace
        self.navigation = navigation
        self.slam = slam
        self.collision_monitor = collision_monitor

        self.lidar: Lidar | None = None
        self.camera: Camera | None = None

        if platform == "panther":
            Vizanti.add_robot_model(self.namespace)
            Vizanti.add_button("Trigger", f"/{self.namespace}/hardware/e_stop_trigger")
            Vizanti.add_button("Reset", f"/{self.namespace}/hardware/e_stop_reset")
            Vizanti.add_button(
                "Estop Status",
                f"/{self.namespace}/hardware/e_stop",
                "std_msgs/msg/Bool",
            )

        if self.navigation or self.slam:
            Rviz.add_map(f"/{self.namespace}/map")

        if self.navigation:
            Rviz.add_map(f"/{self.namespace}/global_costmap/costmap")
            Rviz.add_path(f"/{self.namespace}/plan")
            Vizanti.add_button(
                "Stop", f"/{self.namespace}/waypoint_follower_controller/stop"
            )
            Vizanti.add_initial_pose()
            Vizanti.add_goal_pose()
            Vizanti.add_waypoints(self.namespace)
            Vizanti.add_map(
                "global_costmap", f"/{self.namespace}/global_costmap/costmap"
            )
            Vizanti.add_path(f"/{self.namespace}/plan")

        if self.collision_monitor:
            Rviz.add_polygon(f"/{self.namespace}/polygon_slower")
            Rviz.add_polygon(f"/{self.namespace}/velocity_polygon_stop")

    def create_launch_description(self) -> list[RegisteredLaunchDescription]:
        """Create the launch description with specific elements for a vehicle.

        Returns:
            list[RegisteredLaunchDescription]: The launch description for the platform.
        """
        launch_descriptions = []
        if self.navigation or self.collision_monitor or self.slam:
            launch_descriptions.append(self.create_nav2_launch())
        return launch_descriptions

    def joystick_nodes(self) -> list[Node]:
        """Return the required nodes to control the platform with a joystick.

        Returns:
            list[Node]: The joystick nodes for the platform.
        """
        nodes: list[Node] = []

        pub_topic = f"/{self.namespace}/cmd_vel"
        if self.collision_monitor:
            pub_topic += "_raw"
        if self.platform == "panther":
            nodes.append(
                Node(
                    package="rcdt_joystick",
                    executable="joy_to_twist.py",
                    parameters=[
                        {"sub_topic": f"/{self.namespace}/joy"},
                        {"pub_topic": pub_topic},
                        {"config_pkg": "rcdt_panther"},
                    ],
                    namespace=self.namespace,
                )
            )

        return nodes

    def create_nav2_launch(self) -> RegisteredLaunchDescription:
        """Create the Nav2 launch description.

        Returns:
            RegisteredLaunchDescription: The launch description for the Nav2.

        Raises:
            ValueError: If no lidar is attached to the vehicle.
        """
        if not self.lidar:
            raise ValueError("A lidar is required for use of nav2.")

        return RegisteredLaunchDescription(
            get_file_path("rcdt_panther", ["launch"], "nav2.launch.py"),
            launch_arguments={
                "simulation": str(EnvironmentConfig.simulation),
                "slam": str(self.slam),
                "collision_monitor": str(self.collision_monitor),
                "navigation": str(self.navigation),
                "namespace_vehicle": self.namespace,
                "namespace_lidar": self.lidar.namespace,
            },
        )
