#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
import cv2
from rclpy.node import Node
from rclpy import logging
from rcdt_detection_msgs.srv import SegmentImage
from rcdt_detection.segmentation import segment_image, load_segmentation_model
from rcdt_detection.image_manipulation import (
    cv2_image_to_ros_image,
    ros_image_to_cv2_image,
    segmentation_mask_to_binary_mask,
    single_to_three_channel,
)

ros_logger = logging.get_logger(__name__)


class SegmentImageNode(Node):
    def __init__(self) -> None:
        super().__init__("segment_image")
        self.declare_parameter("model_name", "SAM2")
        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        self.create_service(SegmentImage, "segment_image", self.callback)
        self.model = load_segmentation_model(model_name)

    def callback(
        self, request: SegmentImage.Request, response: SegmentImage.Response
    ) -> SegmentImage.Response:
        input_image_ros = request.input_image
        input_image_cv2 = ros_image_to_cv2_image(input_image_ros)

        ros_logger.info("Start segmentation...")
        result = segment_image(self.model, input_image_cv2)
        ros_logger.info("Finished segmentation!")
        output_image_cv2 = result.plot(labels=False, boxes=False, conf=False)
        output_image_ros = cv2_image_to_ros_image(output_image_cv2)
        response.segmented_image = output_image_ros

        for mask in result.masks:
            single_channel = segmentation_mask_to_binary_mask(mask)
            three_channel = single_to_three_channel(single_channel)
            mask_image_cv2 = cv2.bitwise_and(input_image_cv2, three_channel)
            mask_image_ros = cv2_image_to_ros_image(mask_image_cv2)
            response.masks.append(mask_image_ros)

        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    try:
        node = SegmentImageNode()
        rclpy.spin(node)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
