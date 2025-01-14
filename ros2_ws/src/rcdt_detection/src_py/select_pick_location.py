#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy import logging
from rcdt_detection_msgs.srv import SelectPickLocation as Srv
from rcdt_utilities.cv_utils import (
    ros_image_to_cv2_image,
    cv2_image_to_ros_image,
    camera_info_to_intrinsics,
)
from rcdt_utilities.launch_utils import spin_node
from rcdt_detection.mask_properties import MaskProperties
from geometry_msgs.msg import Point, Quaternion
from tf_transformations import quaternion_from_euler

logger = logging.get_logger(__name__)


class SelectPickLocation(Node):
    def __init__(self) -> None:
        super().__init__("filter_masks")
        self.create_service(Srv, "/select_pick_location", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        depth_image = ros_image_to_cv2_image(request.depth_image)
        intrinsics = camera_info_to_intrinsics(request.camera_info)
        pickup_poses: list[tuple] = []
        visualizations: list[np.ndarray] = []

        for mask_ros in request.masks:
            mask_cv2 = ros_image_to_cv2_image(mask_ros)
            mask_properties = MaskProperties(mask_cv2, depth_image, intrinsics)
            chosen_point2d = mask_properties.chosen_pickup_point
            if chosen_point2d is not None:
                point_3d, eulerangles = mask_properties.point_2d_to_pose(chosen_point2d)
                pickup_poses.append((point_3d, eulerangles))
            visualizations.append(mask_properties.filter_visualization)

        if len(pickup_poses) == 0:
            logger.error("No suitable pick location found.")
            pickup_pose = None
            # Set succes to true in order to inspect visualization in ROSboard
            response.success = False
        else:
            pickup_poses.sort(key=lambda x: x[0].y)
            pickup_pose = pickup_poses[-1]
            response.pick_location.pose.position = Point(
                x=pickup_pose[0].x, y=pickup_pose[0].y, z=pickup_pose[0].z
            )
            quaternion = quaternion_from_euler(*pickup_pose[1])
            response.pick_location.pose.orientation = Quaternion(
                x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
            )
            response.success = True

        response.visualization = cv2_image_to_ros_image(np.max(visualizations, axis=0))
        response.pick_location.header.frame_id = "camera_depth_optical_frame"
        logger.info("returning pick location")
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = SelectPickLocation()
    spin_node(node)


if __name__ == "__main__":
    main()
