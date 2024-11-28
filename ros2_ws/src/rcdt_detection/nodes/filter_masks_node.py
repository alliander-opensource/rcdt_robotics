#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from rclpy import logging
from rcdt_detection_msgs.srv import FilterMasks
from rcdt_detection.object_detectors import is_brick
from rcdt_detection.image_manipulation import ros_image_to_cv2_image

ros_logger = logging.get_logger(__name__)


class FilterMasksNode(Node):
    def __init__(self) -> None:
        super().__init__("filter_masks")
        self.create_service(FilterMasks, "/filter_masks", self.callback)

    def callback(
        self, request: FilterMasks.Request, response: FilterMasks.Response
    ) -> FilterMasks.Response:
        """Segments"""
        match request.filter_method:
            case "brick":
                is_selected = is_brick
            case _:
                ros_logger.warn("Selected filter_method unknown. Exiting.")
                response.success = False
                return response

        n_slected = 0
        for mask_ros in request.masks:
            mask_cv2 = ros_image_to_cv2_image(mask_ros)
            if is_selected(mask_cv2):
                response.masks.append(mask_ros)
                n_slected += 1
        ros_logger.info(
            f"Checked {len(request.masks)} masks, of which {n_slected} are selected."
        )
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = FilterMasksNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
