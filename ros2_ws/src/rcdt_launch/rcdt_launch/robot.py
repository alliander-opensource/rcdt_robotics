# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import math
from typing import Literal

from launch_ros.actions import Node
from rcdt_utilities.launch_utils import get_file_path, get_robot_description
from rcdt_utilities.register import RegisteredLaunchDescription

from rcdt_launch.moveit import Moveit
from rcdt_launch.rviz import Rviz
from rcdt_launch.vizanti import Vizanti


class Platform:  # noqa: PLR0904
    """A class used to dynamically create all the required nodes for a platform.

    Attributes:
        simulation (bool): Whether the platforms are in simulation mode or not.
        platforms (list[Platform]): A list of all the platforms.
        platform_indices (dict[str, int]): A collections of the different platforms and the number of occurrences.
        names (list[str]): A list of all robot names.
        bridge_topics (list[str]): A list of all topics that should be bridged between Gazebo and ROS.
    """

    simulation: bool = True
    platforms: list["Platform"] = []
    platform_indices: dict[str, int] = {}
    names: list[str] = []
    bridge_topics: list[str] = []

    @staticmethod
    def add(platform: "Platform") -> None:
        """Add a platform instance to the general list.

        Args:
            platform (Platform): The platform instance to add.
        """
        Platform.platforms.append(platform)
        current_value = Platform.platform_indices.get(platform.platform, 0)
        Platform.platform_indices[platform.platform] = current_value + 1

    @staticmethod
    def reset() -> None:
        """Reset the platform class to its initial state."""
        Platform.platforms = []
        Platform.platform_indices = {}
        Platform.names = []
        Platform.bridge_topics = []

    @staticmethod
    def generate_namespace(platform: Platform) -> str:
        """Generate an unique namespace for the given platform.

        Args:
            platform (Platform): The platform.

        Returns:
            str: The unique namespace for the platform.
        """
        index = Platform.platform_indices.get(platform.platform, 0)
        return f"{platform.platform}{index}"

    @staticmethod
    def create_state_publishers() -> list[Node]:
        """Create state publisher nodes for all platforms.

        Returns:
            list[Node]: A list of all state publisher nodes.
        """
        nodes = []
        for robot in Platform.platforms:
            node = robot.create_state_publisher()
            if isinstance(node, Node):
                nodes.append(node)
        return nodes

    @staticmethod
    def order_platforms() -> None:
        """Order the platforms list.

        For some reason, the ros control plugin in Gazebo does not load for Franka robots when another platform is loaded before.
        Therefore we can load all Franka robots before other platforms by rearranging the list using this method.
        When launching a vehicle with Nav2, lidar sensor output is required.
        Therefore we load a lidar before a vehicle.

        Raises:
            ValueError: If an unknown platform is encountered.
        """
        order = ["panther", "franka", "velodyne", "realsense", "axis"]

        for platform in Platform.platforms:
            if platform.platform not in order:
                raise ValueError(f"Unknown platform to order: {platform.platform}")

        Platform.platforms = sorted(
            Platform.platforms, key=lambda platform: order.index(platform.platform)
        )

    @staticmethod
    def create_gazebo_launch(load_gazebo_ui: bool) -> RegisteredLaunchDescription:
        """Create the Gazebo launch description.

        Args:
            load_gazebo_ui (bool): Whether to load the Gazebo UI or not.

        Returns:
            RegisteredLaunchDescription: The Gazebo launch description.
        """
        platforms = []
        positions = []
        orientations = []
        parents = []
        parent_links = []

        for platform in Platform.platforms:
            platforms.append(platform.namespace)
            positions.append(",".join(map(str, platform.position)))
            orientations.append(",".join(map(str, platform.orientation)))
            parent = platform.parent
            if parent is not None:
                parents.append(parent.namespace)
                parent_links.append(platform.parent_link)
            else:
                parents.append("none")
                parent_links.append("none")

        return RegisteredLaunchDescription(
            get_file_path("rcdt_gazebo", ["launch"], "gazebo_robot.launch.py"),
            launch_arguments={
                "load_gazebo_ui": str(load_gazebo_ui),
                "platforms": " ".join(platforms),
                "positions": " ".join(positions),
                "orientations": " ".join(orientations),
                "parents": " ".join(parents),
                "parent_links": " ".join(parent_links),
                "bridge_topics": " ".join(Platform.bridge_topics),
            },
        )

    @staticmethod
    def create_hardware_interfaces() -> list[RegisteredLaunchDescription]:
        """Create hardware interface launch descriptions for the platforms.

        Returns:
            list[RegisteredLaunchDescription]: A list of all hardware interface launch descriptions.
        """
        hardware_interfaces = []
        if Platform.simulation:
            return hardware_interfaces

        for robot in Platform.platforms:
            if isinstance(robot, Arm) and robot.platform == "franka":
                hardware_interfaces.append(
                    RegisteredLaunchDescription(
                        get_file_path("rcdt_franka", ["launch"], "robot.launch.py"),
                        launch_arguments={
                            "namespace": robot.namespace,
                            "ip_address": robot.ip_address,
                        },
                    )
                )
        return hardware_interfaces

    @staticmethod
    def create_parent_links() -> list[Node]:
        """Create a list of nodes that link all the platforms to their parent.

        Returns:
            list[Node]: A list of all the required static_transform_publisher nodes.
        """
        nodes = []
        for robot in Platform.platforms:
            if robot.parent is not None:
                nodes.append(robot.create_parent_link())
        return nodes

    @staticmethod
    def create_controllers() -> list[RegisteredLaunchDescription]:
        """Create controllers for all robots.

        Returns:
            list[RegisteredLaunchDescription]: A list of all controller launch descriptions.
        """
        controllers = []
        for robot in Platform.platforms:
            if not Platform.simulation and robot.platform == "panther":
                continue
            if robot.controller_path is not None:
                controllers.append(robot.create_controller())
        return controllers

    @staticmethod
    def create_launch_descriptions() -> list[RegisteredLaunchDescription]:
        """Create launch descriptions for all platforms.

        Returns:
            list[RegisteredLaunchDescription]: A list of all launch descriptions.
        """
        launch_descriptions = []
        for robot in Platform.platforms:
            launch_description = robot.create_launch_description()
            if launch_description != []:
                launch_descriptions.extend(launch_description)
        return launch_descriptions

    @staticmethod
    def create_joystick_nodes() -> list[Node]:
        """Create joystick nodes for all robots.

        Returns:
            list[Node]: A list of all joystick nodes.

        Raises:
            ValueError: If more than one vehicle or arm is linked to the joystick.
        """
        nodes = []

        # At the moment, we only support one vehicle and one arm being linked to the joystick.
        # The vehicle (base) is linked to the B button and the arm to the A button.
        # The X button is always available to stop joystick control of all platforms.
        buttons = [2]  # X button (to stop)
        services = [False]
        topics = [""]
        vehicle_linked = False
        arm_linked = False
        for robot in Platform.platforms:
            button = None
            if isinstance(robot, Vehicle):
                if vehicle_linked:
                    raise ValueError("Only one vehicle can be linked to the joystick.")
                button = 1  # B button (for base)
                vehicle_linked = True
            elif isinstance(robot, Arm):
                if arm_linked:
                    raise ValueError("Only one arm can be linked to the joystick.")
                button = 0  # A button (for arm)
                arm_linked = True
            if button is not None:
                buttons.append(button)
                services.append(False)
                topics.append(f"/{robot.namespace}/joy")
                nodes.extend(robot.joystick_nodes())

        # If none of the platforms require joystick nodes, return:
        if len(nodes) == 0:
            return nodes

        # Add the general joystick node:
        nodes.append(
            Node(
                package="joy",
                executable="game_controller_node",
                parameters=[
                    {"sticky_buttons": True},
                ],
            )
        )

        # Add the joy topic manager node:
        nodes.append(
            Node(
                package="rcdt_joystick",
                executable="joy_topic_manager.py",
                parameters=[
                    {"buttons": buttons},
                    {"services": services},
                    {"topics": topics},
                ],
            )
        )
        return nodes

    @staticmethod
    def create_map_links() -> list[Node]:
        """Create a list of nodes that link all the platforms to the 'map' frame.

        Returns:
            list[Node]: A list of all the static_transform_publisher nodes linking the platforms to the 'map' frame.
        """
        nodes = []
        for robot in Platform.platforms:
            if robot.parent is None:
                node = robot.create_map_link()
                if isinstance(node, Node):
                    nodes.append(node)
        return nodes

    def __init__(  # noqa: PLR0913
        self,
        platform: Literal["panther", "franka", "velodyne", "realsense", "axis"],
        position: list,
        orientation: list | None = None,
        namespace: str | None = None,
        parent: "Platform" | None = None,
        parent_link: str = "",
    ):
        """Initialize a robot instance.

        Args:
            platform (Literal["panther", "franka", "velodyne", "realsense", "axis"]): The platform type of the robot.
            position (list): The initial position of the robot.
            orientation (list | None): The initial orientation of the robot.
            namespace (str | None): The namespace of the robot. If None, a unique namespace will be generated.
            parent (Platform | None): The parent robot, if any.
            parent_link (str): The link of the parent to which the platform is attached. If empty, the base_link of the parent is used.
        """
        self.platform: Literal["panther", "franka", "velodyne", "realsense"] = platform
        self.parent = parent
        self.childs = []
        Platform.add(self)
        self.namespace = namespace if namespace else Platform.generate_namespace(self)

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
            self.parent_link = parent_link if parent_link else parent.base_link
            parent.add_child(self)

    @property
    def robot_description(self) -> dict:
        """Return the robot description for the robot.

        Returns:
            dict: The robot description for the robot.
        """
        xacro_arguments = {
            "simulation": str(Platform.simulation),
            "namespace": self.namespace,
        }
        xacro_arguments["childs"] = str(self.childs)
        xacro_arguments["parent"] = "" if self.is_child else "world"
        if isinstance(self, Arm) and self.platform == "franka":
            xacro_arguments["ip_address"] = self.ip_address

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
    def xacro_path(self) -> str:
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
            case "realsense":
                return get_file_path(
                    "rcdt_sensors", ["urdf"], "rcdt_realsense_d435.urdf.xacro"
                )
            case "axis":
                return get_file_path("rcdt_sensors", ["urdf"], "rcdt_axis.urdf.xacro")
            case _:
                raise ValueError("Can't load xacro: unknown platform.")

    @property
    def base_link(self) -> str:
        """Return the base link for the robot based on its platform.

        Returns:
            str: The base link for the robot.

        Raises:
            ValueError: If the platform is unknown.
        """
        match self.platform:
            case "panther":
                return "base_footprint"
            case "franka":
                return "fr3_link0"
            case "velodyne":
                return "base_link"
            case "realsense":
                return "base_link"
            case "axis":
                return "base_link"
            case _:
                raise ValueError("Unable to provide base_link: Unknown platform.")

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
        if not Platform.simulation and self.platform == "panther":
            return None

        return Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=self.namespace,
            parameters=[self.robot_description, {"frame_prefix": self.frame_prefix}],
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

    def create_map_link(self) -> Node | None:
        """Create a static_transform_publisher node that links the platform to the map.

        If the platform is a Vehicle that is using SLAM or navigation, None is returned.
        In this case, a localization node will make the link to the 'map' frame.
        If a vehicle is not using SLAM or navigation, the 'odom' frame is directly linked to the 'map' frame.

        Returns:
            Node | None: A static_transform_publisher node that links the platform with the world or None if not applicable.
        """
        if isinstance(self, Vehicle):
            if self.navigation or self.slam:
                return None
            child_frame = f"{self.namespace}/odom"
        elif self.platform == "franka" and not Platform.simulation:
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
            "simulation": str(Platform.simulation),
            "namespace": self.namespace,
        }
        if isinstance(self, Arm) and self.platform == "franka":
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


class Camera(Platform):
    """Extension on Platform with camera specific functionalities."""

    def __init__(  # noqa: PLR0913
        self,
        platform: Literal["realsense", "zed"],
        position: list,
        orientation: list | None = None,
        namespace: str | None = None,
        parent: Arm | Vehicle | None = None,
        parent_link: str = "",
    ):
        """Initialize the Camera platform.

        Args:
            platform (Literal["realsense", "zed"]): The platform type.
            position (list): The position of the camera.
            orientation (list | None): The initial orientation of the camera.
            namespace (str | None): The namespace of the camera.
            parent (Arm | Vehicle | None): The parent platform.
            parent_link (str): The link of the parent to which the platform is attached. If empty, the base_link of the parent is used.
        """
        super().__init__(
            platform, position, orientation, namespace, parent, parent_link
        )

        if parent:
            parent.camera = self

        Rviz.add_image(f"/{self.namespace}/color/image_raw")
        Rviz.add_image(f"/{self.namespace}/depth/image_rect_raw")
        Rviz.add_depth_cloud(
            f"/{self.namespace}/color/image_raw",
            f"/{self.namespace}/depth/image_rect_raw",
        )

        Platform.bridge_topics.extend(
            [
                f"/{self.namespace}/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                f"/{self.namespace}/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
                f"/{self.namespace}/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                f"/{self.namespace}/depth/image_rect_raw_float@sensor_msgs/msg/Image@gz.msgs.Image",
            ]
        )

    def create_launch_description(self) -> list[RegisteredLaunchDescription]:
        """Create the launch description with specific elements for a camera.

        Returns:
            list[RegisteredLaunchDescription]: The launch description for the platform.
        """
        launch_descriptions = []
        if self.platform == "realsense":
            launch_descriptions.append(self.create_realsense_launch())
        return launch_descriptions

    def create_realsense_launch(self) -> RegisteredLaunchDescription:
        """Create the Realsense launch description.

        Returns:
            RegisteredLaunchDescription: The launch description for the Realsense.
        """
        return RegisteredLaunchDescription(
            get_file_path("rcdt_sensors", ["launch"], "realsense.launch.py"),
            launch_arguments={
                "simulation": str(Platform.simulation),
                "namespace": self.namespace,
            },
        )


class Lidar(Platform):
    """Extension on Platform with lidar specific functionalities."""

    def __init__(  # noqa: PLR0913
        self,
        platform: Literal["velodyne"],
        position: list,
        orientation: list | None = None,
        namespace: str | None = None,
        parent: Vehicle | None = None,
        parent_link: str = "",
    ):
        """Initialize the Lidar platform.

        Args:
            platform (Literal["velodyne"]): The platform type.
            position (list): The position of the lidar.
            orientation (list | None): The initial orientation of the lidar.
            namespace (str | None): The namespace of the lidar.
            parent (Vehicle | None): The parent platform.
            parent_link (str): The link of the parent to which the platform is attached. If empty, the base_link of the parent is used.
        """
        super().__init__(
            platform, position, orientation, namespace, parent, parent_link
        )

        if parent:
            parent.lidar = self

        Rviz.add_point_cloud(self.namespace)
        Rviz.add_laser_scan(self.namespace)
        Vizanti.add_laser_scan(self.namespace)
        Platform.bridge_topics.append(
            f"/{self.namespace}/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"
        )

    def create_launch_description(self) -> list[RegisteredLaunchDescription]:
        """Create the launch description with specific elements for a lidar.

        Returns:
            list[RegisteredLaunchDescription]: The launch description for the platform.
        """
        launch_descriptions = []
        launch_descriptions.append(self.create_velodyne_launch())
        return launch_descriptions

    def create_velodyne_launch(self) -> RegisteredLaunchDescription:
        """Create the Velodyne launch description.

        Returns:
            RegisteredLaunchDescription: The launch description for Velodyne.
        """
        if self.parent:
            target_frame = f"{self.parent.namespace}/{self.parent.base_link}"
        else:
            target_frame = f"{self.namespace}/base_link"

        return RegisteredLaunchDescription(
            get_file_path("rcdt_sensors", ["launch"], "velodyne.launch.py"),
            launch_arguments={
                "simulation": str(Platform.simulation),
                "namespace": self.namespace,
                "target_frame": target_frame,
            },
        )


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
            Rviz.add_motion_planning_plugin(self.namespace)

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
                "robot_name": "fr3",
                "moveit_package_name": "rcdt_franka_moveit_config",
                "servo_params_package": "rcdt_franka",
                "namespace": self.namespace,
            },
        )


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
                "simulation": str(Platform.simulation),
                "slam": str(self.slam),
                "collision_monitor": str(self.collision_monitor),
                "navigation": str(self.navigation),
                "namespace_vehicle": self.namespace,
                "namespace_lidar": self.lidar.namespace,
            },
        )
