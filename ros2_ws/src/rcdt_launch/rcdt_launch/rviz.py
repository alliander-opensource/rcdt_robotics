# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import yaml
from rcdt_utilities.launch_utils import get_file_path, get_yaml


class Rviz:
    """A class to dynammically manage the RViz configuration.

    Attributes:
        yaml (dict): The default RViz configuration.
        displays (list): The list of displays in the RViz configuration.
        load_motion_planning_plugin (bool): Whether to load the motion planning plugin.
        load_point_cloud (bool): Whether to load point cloud displays.
        moveit_namespaces (list[str]): A list of the namespaces where MoveIt is launched.
    """

    yaml: dict = get_yaml(get_file_path("rcdt_utilities", ["rviz"], "default.rviz"))
    displays: list = yaml["Visualization Manager"]["Displays"]
    load_motion_planning_plugin: bool = False
    load_point_cloud: bool = False
    moveit_namespaces: list[str] = []

    @staticmethod
    def set_fixed_frame(frame: str) -> None:
        """Set the fixed frame.

        Args:
            frame (str): The fixed frame to set.
        """
        Rviz.yaml["Visualization Manager"]["Global Options"]["Fixed Frame"] = frame

    @staticmethod
    def set_grid_height(height: float) -> None:
        """Set the grid height.

        Args:
            height (float): The grid height to set.
        """
        Rviz.displays[0]["Offset"]["Z"] = height

    @staticmethod
    def add_robot_model(namespace: str) -> None:
        """Add a robot model to the RViz configuration.

        Args:
            namespace (str): The namespace of the robot.
        """
        Rviz.displays.append(
            {
                "Enabled": True,
                "Class": "rviz_default_plugins/RobotModel",
                "Description Topic": {"Value": f"/{namespace}/robot_description"},
                "TF Prefix": namespace,
                "Name": namespace,
            }
        )

    @staticmethod
    def add_image(topic: str) -> None:
        """Add an image to the RViz configuration.

        Args:
            topic (str): The topic of the image.
        """
        Rviz.displays.append(
            {
                "Enabled": True,
                "Class": "rviz_default_plugins/Image",
                "Topic": {"Value": topic},
                "Name": topic,
            }
        )

    @staticmethod
    def add_depth_cloud(color_topic: str, depth_topic: str) -> None:
        """Add a depth cloud to the RViz configuration.

        Args:
            color_topic (str): The topic of the color image.
            depth_topic (str): The topic of the depth image.
        """
        Rviz.displays.append(
            {
                "Enabled": True,
                "Class": "rviz_default_plugins/DepthCloud",
                "Color Image Topic": color_topic,
                "Depth Map Topic": depth_topic,
                "Name": depth_topic,
            }
        )

    @staticmethod
    def add_point_cloud(namespace: str) -> None:
        """Add a point cloud to the RViz configuration.

        Args:
            namespace (str): The namespace of the robot.
        """
        if not Rviz.load_point_cloud:
            return

        Rviz.displays.append(
            {
                "Enabled": True,
                "Class": "rviz_default_plugins/PointCloud2",
                "Topic": {"Value": f"/{namespace}/scan/points"},
                "Name": namespace,
            }
        )

    @staticmethod
    def add_laser_scan(namespace: str) -> None:
        """Add a laser scan to the RViz configuration.

        Args:
            namespace (str): The namespace of the robot.
        """
        Rviz.displays.append(
            {
                "Enabled": True,
                "Class": "rviz_default_plugins/LaserScan",
                "Topic": {
                    "Value": f"/{namespace}/scan",
                    "Reliability Policy": "Best Effort",
                },
                "Name": namespace,
                "Color Transformer": "FlatColor",
                "Color": "255; 0; 0",
                "Size (m)": 0.02,
            }
        )

    @staticmethod
    def add_motion_planning_plugin(namespace: str) -> None:
        """Add the motion planning plugin to the RViz configuration.

        Args:
            namespace (str): The namespace of the robot.
        """
        if not Rviz.load_motion_planning_plugin:
            return
        Rviz.moveit_namespaces.append(namespace)
        Rviz.displays.append(
            {
                "Enabled": True,
                "Class": "moveit_rviz_plugin/MotionPlanning",
                "Move Group Namespace": namespace,
                "Robot Description": f"{namespace}_robot_description",
                "Name": f"{namespace}_motion_planning",
                "Planning Scene Topic": f"/{namespace}/monitored_planning_scene",
            }
        )

    @staticmethod
    def add_map(topic: str) -> None:
        """Add a map to the RViz configuration.

        Args:
            topic (str): The topic of the map.
        """
        color_scheme = "costmap" if "costmap" in topic else "map"
        Rviz.displays.append(
            {
                "Enabled": True,
                "Class": "rviz_default_plugins/Map",
                "Name": topic,
                "Topic": {"Value": topic},
                "Color Scheme": color_scheme,
            }
        )

    @staticmethod
    def add_path(topic: str) -> None:
        """Add a path to the RViz configuration.

        Args:
            topic (str): The topic of the path.
        """
        Rviz.displays.append(
            {
                "Enabled": True,
                "Class": "rviz_default_plugins/Path",
                "Name": topic,
                "Topic": {"Value": topic},
            }
        )

    @staticmethod
    def add_polygon(topic: str) -> None:
        """Add a polygon to the RViz configuration.

        Args:
            topic (str): The topic of the polygon.
        """
        Rviz.displays.append(
            {
                "Enabled": True,
                "Class": "rviz_default_plugins/Polygon",
                "Name": topic,
                "Topic": {"Value": topic},
                "Color": "25; 255; 0",
            }
        )

    @staticmethod
    def create_rviz_file() -> None:
        """Create the RViz configuration file."""
        with open("/tmp/rviz.rviz", "w", encoding="utf-8") as outfile:
            yaml.dump(Rviz.yaml, outfile, default_flow_style=False)
