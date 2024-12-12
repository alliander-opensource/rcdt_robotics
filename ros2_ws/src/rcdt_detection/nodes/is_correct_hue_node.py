#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import colorsys
import rclpy
import numpy as np
from rclpy import logging
from rclpy.node import Node

from rcdt_detection_msgs.srv import IsCorrectHue
from rcdt_utilities.cv_utils import ros_image_to_cv2_image

ros_logger = logging.get_logger(__name__)


class IsCorrectHueNode(Node):
    def __init__(self) -> None:
        super().__init__("is_correct_hue")
        self.create_service(IsCorrectHue, "/is_correct_hue", self.callback)

    def callback(
        self, request: IsCorrectHue.Request, response: IsCorrectHue.Response
    ) -> IsCorrectHue.Response:
        """Determine if average hue of image is within given bounds.
        
        Always returns success=True.

        """
        cv2_image = ros_image_to_cv2_image(request.image)

        b_mean, g_mean, r_mean = np.nanmean(np.where(cv2_image==0.0, np.nan, cv2_image), axis=(0,1))
        hue_mean, *_ = colorsys.rgb_to_hsv(r_mean, g_mean, b_mean)
        hue_mean = hue_mean * 360
        ros_logger.debug(f"Mean hue calculated: {hue_mean}")

        response.is_correct_hue = True if request.lower_limit < hue_mean < request.upper_limit else False
        response.success = True
        ros_logger.debug(f"Correct hue: {response.is_correct_hue}")
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    try:
        node = IsCorrectHueNode()
        rclpy.spin(node)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
