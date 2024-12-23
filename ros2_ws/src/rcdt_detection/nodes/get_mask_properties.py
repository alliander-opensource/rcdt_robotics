#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy import logging
from rclpy.node import Node
from rcdt_utilities.launch_utils import spin_node

from vision_msgs.msg import Point2D
from geometry_msgs.msg import Point, PointStamped

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
        for point in mask_properties.box_contour_2d:
            point_2d = Point2D(x=float(point[0]), y=float(point[1]))
            response.box_contour_2d.append(point_2d)
        for point in mask_properties.box_contour_3d:
            point_3d = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            point_stamped = PointStamped()
            point_stamped.header.frame_id = "camera_depth_optical_frame"
            point_stamped.point = point_3d
            response.box_contour_3d.append(point_stamped)
        response.box_shape_3d = mask_properties.box_shape_3d

        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = GetMaskPropertiesNode()
    spin_node(node)


if __name__ == "__main__":
    main()
