#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import cv2
import rclpy
from rcdt_detection.image_manipulation import (
    segmentation_mask_to_binary_mask,
    single_to_three_channel,
)
from rcdt_detection.segmentation import load_segmentation_model, segment_image
from rcdt_messages.srv import SegmentImage as Srv
from rcdt_utilities.cv_utils import cv2_image_to_ros_image, ros_image_to_cv2_image
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node

ros_logger = logging.get_logger(__name__)


class SegmentImage(Node):
    """Node to segment an image using a segmentation model.

    This node provides a service that takes an input image, applies segmentation,
    and returns the segmented image along with individual masks for each segment.

    Attributes:
        node_name (str): The name of the node.
        service_name (str): The name of the service provided by this node.
    """

    def __init__(self) -> None:
        """Initialize the SegmentImage node."""
        super().__init__("segment_image")
        self.declare_parameter("model_name", "SAM2")
        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        self.create_service(Srv, "segment_image", self.callback)
        self.model = load_segmentation_model(model_name)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        """Callback function to handle the service request.

        Args:
            request (Srv.Request): The request containing the input image.
            response (Srv.Response): The response to be filled with the segmented image and masks.

        Returns:
            Srv.Response: The response containing the segmented image and masks.
        """
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
    """Main function to initialize the ROS 2 node and spin it.

    Args:
        args (str, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = SegmentImage()
    spin_node(node)


if __name__ == "__main__":
    main()
