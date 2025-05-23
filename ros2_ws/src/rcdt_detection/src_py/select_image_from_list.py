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
    def __init__(self) -> None:
        super().__init__("select_image_from_list")
        self.create_service(Srv, "/select_image_from_list", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
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
    rclpy.init(args=args)
    node = SelectImageFromList()
    spin_node(node)


if __name__ == "__main__":
    main()
