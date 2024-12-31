#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy import logging
from rclpy.node import Node
from rcdt_utilities.launch_utils import spin_node

from vision_msgs.msg import Point2D

from rcdt_detection_msgs.srv import GetMaskProperties
from rcdt_detection.mask_properties import MaskProperties
from rcdt_utilities.cv_utils import ros_image_to_cv2_image, camera_info_to_intrinsics

ros_logger = logging.get_logger(__name__)


class GetMaskPropertiesNode(Node):
    def __init__(self) -> None:
        super().__init__("get_mask_properties")
        self.create_service(GetMaskProperties, "/get_mask_properties", self.callback)

    def callback(
        self, request: GetMaskProperties.Request, response: GetMaskProperties.Response
    ) -> GetMaskProperties.Response:
        mask = ros_image_to_cv2_image(request.mask)
        depth_image = ros_image_to_cv2_image(request.depth_image)
        intrinsics = camera_info_to_intrinsics(request.camera_info)

        mask_properties = MaskProperties(mask, depth_image, intrinsics)

        for point in mask_properties.contour:
            point_2d = Point2D(x=float(point[0, 0]), y=float(point[0, 1]))
            response.contour.append(point_2d)
        for point in mask_properties.bounding_box.corners:
            point_2d = Point2D(x=float(point.x), y=float(point.y))
            response.box_contour.append(point_2d)
        point = mask_properties.centroid
        response.centroid = Point2D(x=float(point.x), y=float(point.y))
        response.avg_hue = float(mask_properties.avg_hue)

        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = GetMaskPropertiesNode()
    spin_node(node)


if __name__ == "__main__":
    main()
