#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
import pyrealsense2 as rs2
from rclpy import logging
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rcdt_detection_msgs.srv import PoseFromPixel
from rcdt_detection.image_manipulation import ros_image_to_cv2_image

ros_logger = logging.get_logger(__name__)


class PoseFromPixelNode(Node):
    """Converts a pixel to the corresponding pose in meters."""

    def __init__(self) -> None:
        super().__init__("point_from_pixel")
        self.create_service(PoseFromPixel, "/point_from_pixel", self.callback)

    def callback(
        self, request: PoseFromPixel.Request, response: PoseFromPixel.Response
    ) -> PoseFromPixel.Response:
        cv2_image = ros_image_to_cv2_image(request.depth_image)
        height, width = cv2_image.shape
        intr = calculate_intrinsics(request.camera_info)
        x_pixel = int(request.pixel.x)
        y_pixel = int(request.pixel.y)

        if not is_pixel_valid(width, height, [x_pixel, y_pixel]):
            ros_logger.error("Requested pixel location is invalid. Exiting.")
            response.success = False
            return response

        depth_value = cv2_image[y_pixel, x_pixel]
        x_mm, y_mm, z_mm = rs2.rs2_deproject_pixel_to_point(
            intr, [x_pixel, y_pixel], depth_value
        )
        x_m, y_m, z_m = x_mm / 1000, y_mm / 1000, z_mm / 1000
        self.get_logger().info(
            f"Pixel ({x_pixel}, {y_pixel}) corresponds to Point ({x_m:.3f}, {y_m:.3f}, {z_m:.3f})."
        )
        response.pose.pose.position.x = float(x_m)
        response.pose.pose.position.y = float(y_m)
        response.pose.pose.position.z = float(z_m)
        response.pose.header.frame_id = "camera_depth_optical_frame"
        response.success = True
        return response


def is_pixel_valid(width: int, height: int, pixel: list[int, int]) -> bool:
    valid = True
    if not 0 <= pixel[0] < width:
        ros_logger.warn(f"Pixel x={pixel[0]} while image width={width}.")
        valid = False
    if not 0 <= pixel[1] < height:
        ros_logger.warn(f"Pixel y={pixel[0]} while image height={height}.")
        valid = False
    return valid


def realsense_435_intrinsics(width: int, height: int) -> rs2.intrinsics:
    """Gives the intrinsics for a realsense 435."""
    intrinsics = rs2.intrinsics()

    intrinsics.width = int(width)
    intrinsics.height = int(height)
    intrinsics.ppx = 316.3547668457031
    intrinsics.ppy = 243.0697021484375
    intrinsics.fx = 387.0155334472656
    intrinsics.fy = 387.0155334472656

    intrinsics.model = rs2.distortion.brown_conrady

    return intrinsics


def calculate_intrinsics(camera_info: CameraInfo) -> rs2.intrinsics:
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
    try:
        node = PoseFromPixelNode()
        rclpy.spin(node)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
