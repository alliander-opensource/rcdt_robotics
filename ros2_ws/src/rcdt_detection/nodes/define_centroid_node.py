#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
import cv2
from rclpy import logging
from rclpy.node import Node
from rcdt_detection_msgs.srv import DefineCentroid
from rcdt_utilities.cv_utils import ros_image_to_cv2_image, cv2_image_to_ros_image
from rcdt_detection.image_manipulation import three_to_single_channel

ros_logger = logging.get_logger(__name__)


class DefineCentroidNode(Node):
    def __init__(self) -> None:
        super().__init__("define_centroid")
        self.create_service(DefineCentroid, "/define_centroid", self.callback)

    def callback(
        self, request: DefineCentroid.Request, response: DefineCentroid.Response
    ) -> DefineCentroid.Response:
        cv2_image = ros_image_to_cv2_image(request.image)
        single_channel = three_to_single_channel(cv2_image)
        moments = cv2.moments(single_channel)
        x = float(moments["m10"] / moments["m00"])
        y = float(moments["m01"] / moments["m00"])
        radius = int(0.01 * max(cv2_image.shape))
        cv2_marked = cv2.circle(cv2_image, (int(x), int(y)), radius, (255, 0, 0), -1)

        response.centroid.x = x
        response.centroid.y = y
        response.image = cv2_image_to_ros_image(cv2_marked)

        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = DefineCentroidNode()

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
