#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import cv2
import rclpy
import numpy as np
from rclpy import logging
from rclpy.node import Node

from rcdt_detection_msgs.srv import IsRectangleShape
from rcdt_detection.image_manipulation import three_to_single_channel
from rcdt_utilities.cv_utils import ros_image_to_cv2_image, cv2_image_to_ros_image

ros_logger = logging.get_logger(__name__)


class IsRectangleShapeNode(Node):
    def __init__(self) -> None:
        super().__init__("is_rectangle_shape")
        self.create_service(IsRectangleShape, "/is_rectangle_shape", self.callback)

    def callback(
        self, request: IsRectangleShape.Request, response: IsRectangleShape.Response
    ) -> IsRectangleShape.Response:
        """Determine if largest contour is image is a rectangle.

        The given threshold should be in range [0.0-1.0] and determines what fraction
        the area of the contour and a fitted rectangle may differ.

        Returns success=False if given threshold is outside bounds, or no
        contours were found in the image.

        """
        threshold = request.threshold
        if 0.0 < threshold < 1.0:
            ros_logger(f"Given threshold '{threshold}' outside bounds (0.0-1.0).")
            response.success = False
            return response

        cv2_image = ros_image_to_cv2_image(request.image)
        single_channel = three_to_single_channel(cv2_image)

        contours, _ = cv2.findContours(single_channel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            ros_logger("No contours found.")
            response.success = False
            return response
        countour_areas = [cv2.contourArea(contour) for contour in contours]
        largest_contour = contours[countour_areas.index(max(countour_areas))]

        area = cv2.contourArea(largest_contour)
        (x, y), (w, h), angle = cv2.minAreaRect(largest_contour)
        is_rectangle_shape = abs(area - (w * h)) < (area * threshold)
        ros_logger.debug(f"Rectangle shape: {is_rectangle_shape}")

        box = np.int32(cv2.boxPoints(((x, y), (w, h), angle)))
        marked_image = cv2.drawContours(cv2_image, [box], 0, (255, 0, 255), 2)

        response.image = cv2_image_to_ros_image(marked_image)
        response.is_rectangle_shape = True if is_rectangle_shape else False
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    try:
        node = IsRectangleShapeNode()
        rclpy.spin(node)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
