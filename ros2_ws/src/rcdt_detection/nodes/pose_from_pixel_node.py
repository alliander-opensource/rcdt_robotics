#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
import numpy as np
import pyrealsense2 as rs2
from rclpy import logging
from rclpy.node import Node
from rcdt_utilities.launch_utils import spin_node
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Point2D
from geometry_msgs.msg import PointStamped, Point
from rcdt_detection_msgs.srv import PoseFromPixel, ContourTo3D
from rcdt_utilities.cv_utils import ros_image_to_cv2_image

ros_logger = logging.get_logger(__name__)
Pixel = list[int, int]


class PoseFromPixelNode(Node):
    """Converts a pixel to the corresponding pose in meters."""

    def __init__(self) -> None:
        super().__init__("pose_from_pixel")
        self.create_service(PoseFromPixel, "/pose_from_pixel", self.pose_from_pixel)
        self.create_service(ContourTo3D, "/contour_to_3d", self.contour_to_3d)

    def pose_from_pixel(
        self, request: PoseFromPixel.Request, response: PoseFromPixel.Response
    ) -> PoseFromPixel.Response:
        cv2_image = ros_image_to_cv2_image(request.depth_image)
        intrinsics = calculate_intrinsics(request.camera_info)

        if not is_point_valid(cv2_image, request.pixel):
            ros_logger.error("Requested pixel location is invalid. Exiting.")
            response.success = False
            return response

        # TODO: make this neater
        point_stamped = get_point_3d(cv2_image, intrinsics, request.pixel)
        response.pose.pose.position = point_stamped.point
        response.pose.header = point_stamped.header
        response.success = True
        return response

    def contour_to_3d(
        self, request: ContourTo3D.Request, response: ContourTo3D.Response
    ) -> ContourTo3D.Response:
        cv2_image = ros_image_to_cv2_image(request.depth_image)

        if not all(is_point_valid(cv2_image, point) for point in request.contour_2d):
            ros_logger.error("Not all points are valid. Exiting.")
            response.success = False
            return response

        intrinsics = calculate_intrinsics(request.camera_info)

        for point in request.contour_2d:
            response.contour_3d.append(get_point_3d(cv2_image, intrinsics, point))
        response.success = True
        return response


def get_point_3d(
    image: np.ndarray, intrinsics: rs2.intrinsics, point_2d: Point2D
) -> PointStamped:
    x = int(point_2d.x)
    y = int(point_2d.y)
    depth_value = image[y, x]
    x_mm, y_mm, z_mm = rs2.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth_value)
    x_m, y_m, z_m = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0
    point = Point(x=x_m, y=y_m, z=z_m)
    point_stamped = PointStamped(point=point)
    point_stamped.header.frame_id = "camera_depth_optical_frame"
    ros_logger.info(f"point: {point_2d} was transformed to: {[point]}")
    return point_stamped


def is_point_valid(image: np.ndarray, point: Point2D) -> bool:
    height, width = image.shape
    valid = True
    if not 0 <= point.x < width:
        ros_logger.warn(f"point x={point.x} while image width={width}.")
        valid = False
    if not 0 <= point.y < height:
        ros_logger.warn(f"point y={point.y} while image height={height}.")
        valid = False
    return valid


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
    node = PoseFromPixelNode()
    spin_node(node)


if __name__ == "__main__":
    main()
