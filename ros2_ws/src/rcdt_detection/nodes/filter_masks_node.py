#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy import logging
from vision_msgs.msg import Point2D
from rcdt_detection_msgs.srv import FilterMasks
from rcdt_utilities.cv_utils import (
    ros_image_to_cv2_image,
    cv2_image_to_ros_image,
    camera_info_to_intrinsics,
)
from rcdt_utilities.launch_utils import spin_node
from rcdt_detection.mask_properties import MaskProperties

ros_logger = logging.get_logger(__name__)


class FilterMasksNode(Node):
    def __init__(self) -> None:
        super().__init__("filter_masks")
        self.create_service(FilterMasks, "/filter_masks", self.callback)

    def callback(
        self, request: FilterMasks.Request, response: FilterMasks.Response
    ) -> FilterMasks.Response:
        depth_image = ros_image_to_cv2_image(request.depth_image)
        intrinsics = camera_info_to_intrinsics(request.camera_info)

        checked_masks = []
        gripper_locations = []

        for mask_ros in request.masks:
            mask_cv2 = ros_image_to_cv2_image(mask_ros)
            mask_properties = MaskProperties(mask_cv2, depth_image, intrinsics)
            checked_mask, locations = mask_properties.suitable_pick_locations
            checked_masks.append(checked_mask)
            gripper_locations.extend(locations)

        if len(gripper_locations) == 0:
            ros_logger.error("No suitable gripper location found.")
            return response
        gripper_locations.sort(key=lambda location: location[1], reverse=True)

        response.check_result = cv2_image_to_ros_image(np.max(checked_masks, axis=0))
        response.gripper_location = Point2D(
            x=gripper_locations[0][0],
            y=gripper_locations[0][1],
        )
        ros_logger.info(f"Pick locations: {response.gripper_location}")
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = FilterMasksNode()
    spin_node(node)


if __name__ == "__main__":
    main()
