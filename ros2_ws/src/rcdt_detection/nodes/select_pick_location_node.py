#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy import logging
from rcdt_detection_msgs.srv import SelectPickLocation
from rcdt_utilities.cv_utils import (
    ros_image_to_cv2_image,
    cv2_image_to_ros_image,
    camera_info_to_intrinsics,
)
from rcdt_utilities.launch_utils import spin_node
from rcdt_detection.mask_properties import MaskProperties, Point2D, Point3D
from geometry_msgs.msg import Point

ros_logger = logging.get_logger(__name__)


class SelectPickLocationNode(Node):
    def __init__(self) -> None:
        super().__init__("filter_masks")
        self.create_service(SelectPickLocation, "/select_pick_location", self.callback)

    def callback(
        self, request: SelectPickLocation.Request, response: SelectPickLocation.Response
    ) -> SelectPickLocation.Response:
        depth_image = ros_image_to_cv2_image(request.depth_image)
        intrinsics = camera_info_to_intrinsics(request.camera_info)

        points_2d: list[Point2D] = []
        points_3d: list[Point3D] = []
        visualizations: list[np.ndarray] = []

        for mask_ros in request.masks:
            mask_cv2 = ros_image_to_cv2_image(mask_ros)
            mask_properties = MaskProperties(mask_cv2, depth_image, intrinsics)
            point_2d = mask_properties.pick_location
            if point_2d is not None:
                points_2d.append(point_2d)
                points_3d.append(mask_properties.point_2d_to_3d(point_2d))
            visualizations.append(mask_properties.filter_visualization)

        if len(points_2d) == 0:
            ros_logger.error("No suitable pick location found.")
            return response

        index = sorted(
            range(len(points_2d)), key=lambda n: points_2d[n].y, reverse=True
        )[0]
        point = points_3d[index]

        response.pick_location.pose.position = Point(x=point.x, y=point.y, z=point.z)
        response.pick_location.header.frame_id = "camera_depth_optical_frame"
        response.visualization = cv2_image_to_ros_image(np.max(visualizations, axis=0))
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = SelectPickLocationNode()
    spin_node(node)


if __name__ == "__main__":
    main()
