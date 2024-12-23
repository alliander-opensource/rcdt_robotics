#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from rclpy import logging
from rcdt_detection_msgs.srv import FilterMasks
from rcdt_utilities.cv_utils import ros_image_to_cv2_image, camera_info_to_intrinsics
from rcdt_utilities.launch_utils import spin_node
from rcdt_detection.mask_properties import MaskProperties

ros_logger = logging.get_logger(__name__)

BRICK = {"min_width": 0.04, "max_width": 0.065, "min_length": 0.20, "max_length": 0.25}


class FilterMasksNode(Node):
    def __init__(self) -> None:
        super().__init__("filter_masks")
        self.create_service(FilterMasks, "/filter_masks", self.callback)

    def callback(
        self, request: FilterMasks.Request, response: FilterMasks.Response
    ) -> FilterMasks.Response:
        depth_image = ros_image_to_cv2_image(request.depth_image)
        intrinsics = camera_info_to_intrinsics(request.camera_info)

        for mask_ros in request.masks:
            mask_cv2 = ros_image_to_cv2_image(mask_ros)
            mask_properties = MaskProperties(mask_cv2, depth_image, intrinsics)
            if is_brick(mask_properties):
                response.masks.append(mask_ros)

        ros_logger.info(f"{len(response.masks)} masks matched the filter criteria.")
        response.success = True
        return response


def is_brick(mask_properties: MaskProperties) -> bool:
    shape = mask_properties.box_shape_3d
    correct_width = BRICK["min_width"] < shape[0] < BRICK["max_width"]
    correct_length = BRICK["min_length"] < shape[1] < BRICK["max_length"]
    return correct_width and correct_length


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = FilterMasksNode()
    spin_node(node)


if __name__ == "__main__":
    main()
