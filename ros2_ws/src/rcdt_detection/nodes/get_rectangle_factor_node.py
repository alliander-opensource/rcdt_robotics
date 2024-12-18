#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import cv2
import rclpy
import numpy as np
from rclpy import logging
from rclpy.node import Node
from rcdt_utilities.launch_utils import start_node

from rcdt_detection_msgs.srv import GetRectangleFactor

ros_logger = logging.get_logger(__name__)


class GetRectangleFactorNode(Node):
    def __init__(self) -> None:
        super().__init__("get_rectangle_factor")
        self.create_service(GetRectangleFactor, "/get_rectangle_factor", self.callback)

    def callback(
        self, request: GetRectangleFactor.Request, response: GetRectangleFactor.Response
    ) -> GetRectangleFactor.Response:
        """Determine rectangle factor (100 / mask_area * bounding_box_area).

        Always returns success=True.

        """
        mask_contour = np.array(
            [[[point.x, point.y]] for point in request.mask_contour], dtype=np.int32
        )
        bounding_box_contour = np.array(
            [[[point.x, point.y]] for point in request.bounding_box_contour],
            dtype=np.int32,
        )

        mask_area = cv2.contourArea(mask_contour)
        bounding_box_area = cv2.contourArea(bounding_box_contour)
        rectangle_factor = 100 / mask_area * bounding_box_area
        ros_logger.info(f"Rectangle factor: {rectangle_factor}")

        response.factor = rectangle_factor
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = GetRectangleFactorNode()
    start_node(node)


if __name__ == "__main__":
    main()
