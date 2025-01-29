#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import rclpy
from rcdt_detection.mask_properties import MaskProperties, Pose
from rcdt_detection_msgs.srv import SelectPickLocation as Srv
from rcdt_utilities.cv_utils import (
    camera_info_to_intrinsics,
    cv2_image_to_ros_image,
    ros_image_to_cv2_image,
)
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node

logger = logging.get_logger(__name__)


class SelectPickLocation(Node):
    def __init__(self) -> None:
        super().__init__("filter_masks")
        self.create_service(Srv, "/select_pick_location", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        depth_image = ros_image_to_cv2_image(request.depth_image)
        intrinsics = camera_info_to_intrinsics(request.camera_info)
        pickup_poses: list[Pose] = []
        visualizations: list[np.ndarray] = []

        for mask_ros in request.masks:
            mask_cv2 = ros_image_to_cv2_image(mask_ros)
            mask_properties = MaskProperties(mask_cv2, depth_image, intrinsics)
            chosen_point2d = mask_properties.chosen_pickup_point
            if chosen_point2d is not None:
                pose: Pose = mask_properties.point_2d_to_pose(chosen_point2d)
                pickup_poses.append(pose)
            visualizations.append(mask_properties.filter_visualization)

        if len(pickup_poses) == 0:
            logger.error("No suitable pick location found.")
            pickup_pose = None
            response.success = False
        else:
            pickup_poses.sort(key=lambda pose: pose.position.y)
            pickup_pose = pickup_poses[-1]
            response.pick_location.pose = pickup_pose.as_ros_pose()
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
