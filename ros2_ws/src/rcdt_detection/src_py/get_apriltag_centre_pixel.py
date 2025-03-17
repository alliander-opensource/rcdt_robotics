#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rcdt_messages.srv import GetApriltagCentrePixel as Srv
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging, wait_for_message
from rclpy.node import Node
from vision_msgs.msg import Point2D

logger = logging.get_logger(__name__)


class GetApriltagCentrePixel(Node):
    def __init__(self) -> None:
        super().__init__("get_apriltag_centre_pixel")
        self.create_service(Srv, "/get_apriltag_centre_pixel", self.callback)

    def callback(self, _request: Srv.Request, response: Srv.Response) -> Srv.Response:
        logger.info("starting detecting image!")
        response.success, detections = wait_for_message.wait_for_message(
            AprilTagDetectionArray, self, "/detections", time_to_wait=1
        )
        detections: AprilTagDetectionArray
        detection=detections.detections[0]
        if not response.success:
            logger.error("incorrect response received from /detections. Returning.")
            return response
        response.pixel = Point2D(x=detection.centre.x, y=detection.centre.y)


        response.success=True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = GetApriltagCentrePixel()
    spin_node(node)


if __name__ == "__main__":
    main()
