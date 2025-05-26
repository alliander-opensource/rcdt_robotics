#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rcdt_messages.srv import SelectImageFromList as Srv
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node

ros_logger = logging.get_logger(__name__)


class SelectImageFromList(Node):
    """Node to select an image from a list of images.

    This node provides a service that allows the selection of an image from a list
    based on an index provided in the request.

    Attributes:
        node_name (str): The name of the node.
        service_name (str): The name of the service provided by this node.
    """

    def __init__(self) -> None:
        """Initialize the SelectImageFromList node."""
        super().__init__("select_image_from_list")
        self.create_service(Srv, "/select_image_from_list", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        """Callback function to handle the service request.

        Args:
            request (Srv.Request): The request containing the list of images and the index to select.
            response (Srv.Response): The response to be filled with the selected image.

        Returns:
            Srv.Response: The response containing the selected image and success status.
        """
        n_images = len(request.image_list)
        n_select = request.n
        if n_select >= n_images:
            ros_logger.error(
                f"Index {n_select} out of range for length {n_images}. Exit."
            )
            response.success = False
            return response
        response.image = request.image_list[n_select]
        response.success = True
        return response


def main(args: str = None) -> None:
    """Main function to initialize the ROS 2 node and spin it.

    Args:
        args (str, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = SelectImageFromList()
    spin_node(node)


if __name__ == "__main__":
    main()
