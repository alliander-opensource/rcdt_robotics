#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import numpy as np
import rclpy
from rclpy import logging
from rclpy.node import Node
from rcdt_utilities.launch_utils import spin_node
from rcdt_detection_msgs.srv import PublishMasks as Srv
from rcdt_utilities.cv_utils import cv2_image_to_ros_image, ros_image_to_cv2_image
from sensor_msgs.msg import Image

ros_logger = logging.get_logger(__name__)


class PublishMasks(Node):
    def __init__(self) -> None:
        super().__init__("publish_masks")
        self.create_service(Srv, "/publish_masks", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        publisher = self.create_publisher(Image, request.topic, 10)
        masks_cv2 = [ros_image_to_cv2_image(mask) for mask in request.masks]
        if len(masks_cv2) == 0:
            ros_logger.warning("Given input has zero masks. Cancel.")
            return response
        combined = np.max(masks_cv2, axis=0)
        publisher.publish(cv2_image_to_ros_image(combined))
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = PublishMasks()
    spin_node(node)


if __name__ == "__main__":
    main()
