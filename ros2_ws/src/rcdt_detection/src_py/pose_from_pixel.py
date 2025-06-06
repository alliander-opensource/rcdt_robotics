#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import pyrealsense2 as rs2
import rclpy
from rcdt_messages.srv import PoseFromPixel as Srv
from rcdt_utilities.cv_utils import ros_image_to_cv2_image
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

ros_logger = logging.get_logger(__name__)


class PoseFromPixel(Node):
    """Converts a pixel to the corresponding pose in meters.

    This node provides a service that takes a pixel location and depth image,
    and returns the 3D pose of that pixel in the camera's coordinate frame.
    """

    def __init__(self) -> None:
        """Initialize the PoseFromPixel node."""
        super().__init__("pose_from_pixel")
        self.create_service(Srv, "/pose_from_pixel", self.callback)

    def callback(self, request: Srv.Request, response: Srv.Response) -> Srv.Response:
        """Callback function to handle the service request.

        Args:
            request (Srv.Request): The request containing the pixel location and depth image.
            response (Srv.Response): The response to be filled with the pose.

        Returns:
            Srv.Response: The response containing the pose of the pixel in the camera's coordinate frame.
        """
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
    """Check if the pixel coordinates are valid within the image dimensions.

    Args:
        width (int): The width of the image.
        height (int): The height of the image.
        pixel (list[int, int]): The pixel coordinates to validate.

    Returns:
        bool: True if the pixel is valid, False otherwise.
    """
    valid = True
    if not 0 <= pixel[0] < width:
        ros_logger.warn(f"Pixel x={pixel[0]} while image width={width}.")
        valid = False
    if not 0 <= pixel[1] < height:
        ros_logger.warn(f"Pixel y={pixel[0]} while image height={height}.")
        valid = False
    return valid


def calculate_intrinsics(camera_info: CameraInfo) -> rs2.intrinsics:
    """Calculate camera intrinsics for translating image to world coordinates.

    Args:
        camera_info (CameraInfo): The camera information containing intrinsic parameters.

    Returns:
        rs2.intrinsics: The calculated intrinsics for the camera.
    """
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
    """Main function to initialize the ROS 2 node and spin it.

    Args:
        args (str, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = PoseFromPixel()
    spin_node(node)


if __name__ == "__main__":
    main()
