#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
import pyrealsense2 as rs2
from rclpy import logging
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rcdt_detection_msgs.srv import PointFromPixel
from rcdt_detection.image_manipulation import ros_image_to_cv2_image

ros_logger = logging.get_logger(__name__)


class PoseFromPixelNode(Node):
    def __init__(self) -> None:
        super().__init__("point_from_pixel")
        self.create_service(PointFromPixel, "/point_from_pixel", self.callback)

    def callback(
        self, request: PointFromPixel.Request, response: PointFromPixel.Response
    ) -> PointFromPixel.Response:
        cv2_image = ros_image_to_cv2_image(request.depth_image)
        intr = self.calculate_intrinsics(request.camera_info)
        pix_x = int(request.pixel.x)
        pix_y = int(request.pixel.y)
        depth_value = cv2_image[pix_y, pix_x]
        x, y, z = rs2.rs2_deproject_pixel_to_point(intr, [pix_x, pix_y], depth_value)
        self.get_logger().info(
            f"Pixel ({pix_x}, {pix_y}) corresponds to Point ({x:.2f}, {y:.2f}, {z:.2f})."
        )
        response.point.x = float(x)
        response.point.y = float(y)
        response.point.z = float(z)
        response.success = True
        return response

    def calculate_intrinsics(self, camera_info: CameraInfo) -> rs2.intrinsics:
        """Calculate camera intrinsics for translating image to world coordinates."""
        intrinsics = rs2.intrinsics()

        intrinsics.width = camera_info.width
        intrinsics.height = camera_info.height
        intrinsics.ppx = camera_info.k[2]
        intrinsics.ppy = camera_info.k[5]
        intrinsics.fx = camera_info.k[0]
        intrinsics.fy = camera_info.k[4]

        if camera_info.distortion_model == "plumb_bob":
            intrinsics.model = rs2.distortion.brown_conrady
        elif camera_info.distortion_model == "equidistant":
            intrinsics.model = rs2.distortion.kannala_brandt4

        intrinsics.coeffs = list(camera_info.d)

        return intrinsics


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = PoseFromPixelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
