#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from math import pi

import rclpy
from rcdt_messages.srv import SelectPlaceLocation as Srv
from rcdt_utilities.geometry import Point3D, Pose, Quaternion
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node

logger = logging.get_logger(__name__)


class SelectPlaceLocation(Node):
    """Node to select a place location based on a triangular grid.

    This node provides a service that calculates place locations in a triangular grid
    pattern. It starts from a base layer and moves upwards, adjusting the x and y
    coordinates based on the current layer and row index. The pose is returned in a
    ROS-compatible format.

    Attributes:
        node_name (str): The name of the node.
        service_name (str): The name of the service provided by this node.
    """

    def __init__(self) -> None:
        """Initialize the SelectPlaceLocation node."""
        super().__init__("place_locations")
        self.create_service(Srv, "/select_place_location", self.callback)
        self.brick_height = 0.07
        self.brick_offset = 0.08

        self.base_layer_size = 3
        self.layer = 0
        self.row_idx = 0

    def callback(self, _request: Srv.Request, response: Srv.Response) -> Srv.Response:
        """Callback function to handle the service request.

        Args:
            _request (Srv.Request): The request object (not used in this implementation).
            response (Srv.Response): The response to be filled with the place location.

        Returns:
            Srv.Response: The response containing the place location and success status.
        """
        if self.layer >= self.base_layer_size:
            response.success = False
            logger.error("out of triangle layers")
            return response

        x = (
            0.12
            + (self.row_idx * self.brick_offset)
            + (self.layer * (self.brick_offset / 2))
        )
        y = 0.52
        z = 0.0 + (self.layer * self.brick_height)

        rot = 0.2 if self.layer % 2 == 0 else -0.2

        pose = Pose(
            Point3D(x, y, z),
            Quaternion.from_eulerangles(pi, 0.0, (pi / 2) + rot),
        )
        response.place_location.pose = pose.as_ros_pose
        response.success = True
        response.place_location.header.frame_id = "base"
        self.row_idx += 1
        if self.row_idx >= self.base_layer_size - self.layer:
            self.row_idx = 0
            self.layer += 1

        logger.info(f"returning place location: {pose}")
        return response


def main(args: str = None) -> None:
    """Main function to initialize the ROS 2 node and spin it.

    Args:
        args (str, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = SelectPlaceLocation()
    spin_node(node)


if __name__ == "__main__":
    main()
