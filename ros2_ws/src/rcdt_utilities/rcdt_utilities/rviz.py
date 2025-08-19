# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import yaml

from rcdt_utilities.launch_utils import get_file_path, get_yaml


class Rviz:
    """A class to dynammically manage the RViz configuration.

    Attributes:
        yaml (dict): The default RViz configuration.
    """

    yaml: dict = get_yaml(get_file_path("rcdt_utilities", ["rviz"], "default.rviz"))

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
        displays: list = Rviz.yaml["Visualization Manager"]["Displays"]
        displays[0]["Offset"]["Z"] = height

    @staticmethod
    def add_robot_model(namespace: str) -> None:
        """Add a robot model to the RViz configuration.

        Args:
            namespace (str): The namespace of the robot.
        """
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
    def add_point_cloud(namespace: str) -> None:
        """Add a point cloud to the RViz configuration.

        Args:
            namespace (str): The namespace of the robot.
        """
        displays: list = Rviz.yaml["Visualization Manager"]["Displays"]
        displays.append(
            {
                "Alpha": "1",
                "Class": "rviz_default_plugins/PointCloud2",
                "Enabled": "true",
                "Topic": {"Value": f"/{namespace}/scan/points"},
                "Name": f"{namespace}_point_cloud",
            }
        )

    @staticmethod
    def create_rviz_file() -> None:
        """Create the RViz configuration file."""
        with open("/tmp/rviz.rviz", "w", encoding="utf-8") as outfile:
            yaml.dump(Rviz.yaml, outfile, default_flow_style=False)
