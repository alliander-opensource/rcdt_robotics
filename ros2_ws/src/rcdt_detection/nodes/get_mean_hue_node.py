#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import colorsys
import rclpy
import numpy as np
from rclpy import logging
from rclpy.node import Node

from rcdt_detection_msgs.srv import GetMeanHue
from rcdt_utilities.cv_utils import ros_image_to_cv2_image

ros_logger = logging.get_logger(__name__)


class GetMeanHueNode(Node):
    def __init__(self) -> None:
        super().__init__("get_mean_hue")
        self.create_service(GetMeanHue, "/get_mean_hue", self.callback)

    def callback(
        self, request: GetMeanHue.Request, response: GetMeanHue.Response
    ) -> GetMeanHue.Response:
        """Determines average hue of image.
        
        Always returns success=True.

        """
        cv2_image = ros_image_to_cv2_image(request.image)

        b_mean, g_mean, r_mean = np.nanmean(np.where(cv2_image==0.0, np.nan, cv2_image), axis=(0,1))
        hue_mean, *_ = colorsys.rgb_to_hsv(r_mean, g_mean, b_mean)
        hue_mean = hue_mean * 360
        ros_logger.info(f"Mean hue calculated: {hue_mean}")

        response.hue = hue_mean
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    try:
        node = GetMeanHueNode()
        rclpy.spin(node)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
