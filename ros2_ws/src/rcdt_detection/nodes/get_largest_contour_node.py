#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import cv2
import rclpy
from rclpy import logging
from rclpy.node import Node
from vision_msgs.msg import Point2D
from rcdt_utilities.launch_utils import start_node

from rcdt_detection_msgs.srv import GetLargestContour
from rcdt_detection.image_manipulation import three_to_single_channel
from rcdt_utilities.cv_utils import ros_image_to_cv2_image, cv2_image_to_ros_image

ros_logger = logging.get_logger(__name__)


class GetLargestContourNode(Node):
    def __init__(self) -> None:
        super().__init__("get_largest_contour")
        self.create_service(GetLargestContour, "/get_largest_contour", self.callback)

    def callback(
        self, request: GetLargestContour.Request, response: GetLargestContour.Response
    ) -> GetLargestContour.Response:
        """Get largest contour from image.

        Returns success=False if no contours were found.

        """
        cv2_image = ros_image_to_cv2_image(request.image)
        single_channel = three_to_single_channel(cv2_image)

        contours, _ = cv2.findContours(
            single_channel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            ros_logger.error("No contours found.")
            response.success = False
            return response
        countour_areas = [cv2.contourArea(contour) for contour in contours]
        largest_contour = contours[countour_areas.index(max(countour_areas))]
        contour = []
        for ((x, y),) in largest_contour:
            point = Point2D()
            point.x = float(x)
            point.y = float(y)
            contour.append(point)
        ros_logger.info("Found largest contour")

        marked_image = cv2.drawContours(
            cv2_image, [largest_contour], -1, (255, 255, 0), 2
        )

        response.contour = contour
        response.image = cv2_image_to_ros_image(marked_image)
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = GetLargestContourNode()
    start_node(node)


if __name__ == "__main__":
    main()
