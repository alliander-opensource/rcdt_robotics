#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import cv2
import rclpy
import numpy as np
from rclpy import logging
from rclpy.node import Node
from vision_msgs.msg import Point2D
from sensor_msgs.msg import Image

from rcdt_detection_msgs.srv import GetBoundingBox2D
from rcdt_utilities.cv_utils import ros_image_to_cv2_image, cv2_image_to_ros_image

ros_logger = logging.get_logger(__name__)


class GetBoundingBox2DNode(Node):
    def __init__(self) -> None:
        super().__init__("get_bounding_box_2d")
        self.create_service(GetBoundingBox2D, "/get_bounding_box_2d", self.callback)

    def callback(
        self, request: GetBoundingBox2D.Request, response: GetBoundingBox2D.Response
    ) -> GetBoundingBox2D.Response:
        """Get closest fitting 2D bounding box of given contour.

        The returned angle (theta in bounding box center) is in degrees. This is because
        we use opencv and that uses degrees instead of radians.

        """
        given_contour = np.array(
            [[[point.x, point.y]] for point in request.contour], dtype=np.int32
        )
        (x, y), (w, h), angle = cv2.minAreaRect(given_contour)
        ros_logger.info(
            f"Found bounding box with: {x=:.1f}px {y=:.1f}px {w=:.1f}px {h=:.1f}px {angle=:.1f}°"
        )

        box = np.int32(cv2.boxPoints(((x, y), (w, h), angle)))
        ros_logger.info(f"Contour is: {box.tolist()}")
        contour = []
        for _x, _y in box:
            point = Point2D()
            point.x = float(_x)
            point.y = float(_y)
            contour.append(point)

        if request.image.data != []:
            cv2_image = ros_image_to_cv2_image(request.image)
            marked_image = cv2.drawContours(cv2_image, [box], 0, (255, 0, 255), 2)
        else:
            marked_image = Image()

        response.bounding_box.center.position.x = x
        response.bounding_box.center.position.y = y
        response.bounding_box.center.theta = angle
        response.bounding_box.size_x = w
        response.bounding_box.size_y = h
        response.contour = contour
        response.image = cv2_image_to_ros_image(marked_image)
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = GetBoundingBox2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        ros_logger.info("Keyboard interrupt, shutting down.\n")
    except Exception as e:
        raise e
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
