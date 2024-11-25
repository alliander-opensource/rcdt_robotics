#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcdt_detection_msgs.srv import SegmentImage
from rcdt_detection.segmentation import segment_image, load_segmentation_model
from rcdt_detection.image_manipulation import (
    cv2_image_to_ros_image,
    ros_image_to_cv2_image,
)

ros_logger = rclpy.logging.get_logger(__name__)


class SegmentImageNode(Node):
    def __init__(self) -> None:
        super().__init__("segment_image")
        self.declare_parameter("model_name", "")
        model_name = self.get_parameter("model_name").get_parameter_value().string_value

        self.create_service(SegmentImage, "segment_image", self.callback)

        self.pub_input = self.create_publisher(Image, "~/input", 10)
        self.pub_output = self.create_publisher(Image, "~/output", 10)
        self.model = load_segmentation_model(model_name)

    def callback(
        self, request: SegmentImage.Request, response: SegmentImage.Response
    ) -> SegmentImage.Response:
        """Segments"""
        input_image = request.image

        if request.visualize:
            self.pub_input.publish(input_image)

        ros_logger.info("Start segmentation...")
        result = segment_image(self.model, ros_image_to_cv2_image(input_image))
        ros_logger.info("Finished segmentation!")

        if request.visualize:
            output_image = result.plot(labels=False, boxes=False, conf=False)
            self.pub_output.publish(cv2_image_to_ros_image(output_image))

        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = SegmentImageNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
