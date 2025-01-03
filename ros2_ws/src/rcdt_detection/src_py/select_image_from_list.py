#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy import logging
from rclpy.node import Node
from rcdt_detection_msgs.srv import SelectImageFromList
from rcdt_utilities.launch_utils import spin_node

ros_logger = logging.get_logger(__name__)


class SelectImageFromListNode(Node):
    def __init__(self) -> None:
        super().__init__("select_image_from_list")
        self.create_service(
            SelectImageFromList, "/select_image_from_list", self.callback
        )

    def callback(
        self,
        request: SelectImageFromList.Request,
        response: SelectImageFromList.Response,
    ) -> SelectImageFromList.Response:
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
    node = SelectImageFromListNode()
    spin_node(node)


if __name__ == "__main__":
    main()
