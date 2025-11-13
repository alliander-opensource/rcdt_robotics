# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rcdt_launch.platform import Platform


class EnvironmentConfig:
    """A class used to dynamically create all the required nodes for a platform.

    Attributes:
        simulation (bool): Whether the platforms are in simulation mode or not.
        world (str): The world file to be used in Gazebo.
        platforms (list[Platform]): A list of all the platforms.
        platform_indices (dict[str, int]): A collections of the different platforms and the number of occurrences.
        names (list[str]): A list of all robot names.
        bridge_topics (list[str]): A list of all topics that should be bridged between Gazebo and ROS.
    """

    simulation: bool = True
    world: str = "walls.sdf"
    platforms: list["Platform"] = []
    platform_indices: dict[str, int] = {}
    names: list[str] = []
    bridge_topics: list[str] = []
