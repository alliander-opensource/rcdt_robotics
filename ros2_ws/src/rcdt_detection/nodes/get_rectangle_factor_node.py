#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import cv2
import rclpy
import numpy as np
from rclpy import logging
from rclpy.node import Node

from rcdt_detection_msgs.srv import GetRectangleFactor
from rcdt_detection.image_manipulation import three_to_single_channel
from rcdt_utilities.cv_utils import ros_image_to_cv2_image, cv2_image_to_ros_image

ros_logger = logging.get_logger(__name__)


class GetRectangleFactorNode(Node):
    def __init__(self) -> None:
        super().__init__("get_rectangle_factor")
        self.create_service(GetRectangleFactor, "/get_rectangle_factor", self.callback)

    def callback(
        self, request: GetRectangleFactor.Request, response: GetRectangleFactor.Response
    ) -> GetRectangleFactor.Response:
        """Determine rectangle factor of largest contour in image.

        The rectangle factor is: 100 / contour_area * contour_area. The contour
        area is based on the closest-fitting rectangle around the contour.

        Returns success=False if no contours were found.

        """
        cv2_image = ros_image_to_cv2_image(request.image)
        single_channel = three_to_single_channel(cv2_image)

        contours, _ = cv2.findContours(single_channel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            ros_logger.error("No contours found.")
            response.success = False
            return response
        countour_areas = [cv2.contourArea(contour) for contour in contours]
        largest_contour = contours[countour_areas.index(max(countour_areas))]

        area = cv2.contourArea(largest_contour)
        (x, y), (w, h), angle = cv2.minAreaRect(largest_contour)
        rectangle_factor = 100 / area * (w * h)
        ros_logger.info(f"Rectangle factor: {rectangle_factor}")

        box = np.int32(cv2.boxPoints(((x, y), (w, h), angle)))
        marked_image = cv2.drawContours(cv2_image, [box], 0, (255, 0, 255), 2)

        response.image = cv2_image_to_ros_image(marked_image)
        response.factor = rectangle_factor
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    try:
        node = GetRectangleFactorNode()
        rclpy.spin(node)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
