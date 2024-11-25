#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import dataclasses
from typing import Callable, Iterable

import cv2
import numpy as np
import pyrealsense2 as rs2
import rclpy
import ultralytics
from geometry_msgs.msg import Point
from realsense2_camera_msgs.msg import RGBD
from rclpy import wait_for_message
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

from rcdt_detection.image_manipulation import (
    ros_image_to_cv2_image_sliced,
    segmentation_mask_to_binary_mask,
    single_to_three_channel,
)
from rcdt_detection.object_detectors import is_brick
from rcdt_detection.segmentation import (
    load_segmentation_model,
    segment_image,
)
from rcdt_detection_msgs.msg import PointList
from rcdt_detection_msgs.srv import DetectObjects

ros_logger = rclpy.logging.get_logger(__name__)


@dataclasses.dataclass
class ImageCoordinate:
    """Image coordinate where (0, 0) is at the top-left corner."""

    x: int
    y: int


class DetectionService(Node):
    def __init__(self, is_object_of_interest: Callable):
        super().__init__("detection_service")

        self.is_object_of_interest: Callable = is_object_of_interest
        self.segmentation_model: ultralytics.engine.model.Model = (
            load_segmentation_model()
        )
        self.intrinsics: rs2.intrinsics | None = None

        self.rgbd_topic: str = "/camera/camera/rgbd"
        self.time_to_wait: float = 2.0
        self.max_attempts: int = 3
        self.service = self.create_service(
            DetectObjects, "detect_objects", self.detection_callback
        )

    def detection_callback(self, _: None, response: PointList) -> PointList:
        """Gets RGBD image, detects objects, and returns them."""
        try:
            message = self.get_rgbd()
        except LookupError as e:
            ros_logger.error(f"Exception occurred whilst getting RGBD image: {e}")
            response.objects = PointList(points=[])
            return response

        centroid_image_coordinates = process_rgb_image(
            message=message.rgb,
            segmentation_model=self.segmentation_model,
            is_object_of_interest=self.is_object_of_interest,
        )
        ros_logger.info(f"{centroid_image_coordinates}")

        if self.intrinsics is None:
            self.intrinsics = calculate_intrinsics(message.depth_camera_info)
        world_coordinates = process_depth_image(
            message=message.depth,
            image_coordinates=centroid_image_coordinates,
            intrinsics=self.intrinsics,
        )
        ros_logger.info(f"{world_coordinates}")
        response.objects = PointList(points=world_coordinates)
        ros_logger.info(f"{response}")
        return response

    def get_rgbd(self) -> RGBD:
        """Gets RGBD-message from the rgbd topic.

        Raises:
            LookupError: if after `self.max_attempts` no message could be received.

        """
        attempts = 0
        while attempts < self.max_attempts:
            success, message = wait_for_message.wait_for_message(
                msg_type=RGBD,
                node=self,
                topic=self.rgbd_topic,
                time_to_wait=self.time_to_wait,
            )
            if success:
                break
            attempts += 1
        else:
            raise LookupError("unable to get message from RGBD topic")
        ros_logger.info(
            f"Received RGBD message for frame '{message.header.frame_id}' stamped at {repr(message.header.stamp)}."
        )
        return message


def calculate_intrinsics(message: CameraInfo) -> rs2.intrinsics:
    """Calculate camera intrinsics for translating image to world coordinates."""
    intrinsics = rs2.intrinsics()

    intrinsics.width = message.width
    intrinsics.height = message.height
    intrinsics.ppx = message.k[2]
    intrinsics.ppy = message.k[5]
    intrinsics.fx = message.k[0]
    intrinsics.fy = message.k[4]

    if message.distortion_model == "plumb_bob":
        intrinsics.model = rs2.distortion.brown_conrady
    elif message.distortion_model == "equidistant":
        intrinsics.model = rs2.distortion.kannala_brandt4

    intrinsics.coeffs = list(message.d)

    return intrinsics


def process_rgb_image(
    message: Image,
    segmentation_model: ultralytics.engine.model.Model,
    is_object_of_interest: Callable,
) -> list[ImageCoordinate]:
    """Calculate image coordinates of centroids of detected objects.

    Object detection starts with segmenting the image. The result contains segmentation masks
    per object. For every segmented object it is determined whether it is an object of interest.
    If it is, the image coordinates of its centroid are calculated.

    """
    rgb_image = ros_image_to_cv2_image_sliced(message)
    segmentation_result = segment_image(model=segmentation_model, image=rgb_image)

    centroid_image_coordinates = []
    for mask in segmentation_result.masks:
        single_channel = segmentation_mask_to_binary_mask(mask)
        three_channel = single_to_three_channel(single_channel)
        masked_image = cv2.bitwise_and(rgb_image, three_channel)

        if is_object_of_interest(masked_image):
            centroid = get_centroid_image_coordinates(single_channel)
            centroid_image_coordinates.append(centroid)
    ros_logger.info(f"Detected {len(centroid_image_coordinates)} objects.")

    return centroid_image_coordinates


def process_depth_image(
    message: Image,
    image_coordinates: Iterable[ImageCoordinate],
    intrinsics: rs2.intrinsics,
) -> list[Point]:
    """Calculate world coordinate relative to the camera of image coordinates."""
    depth_image = ros_image_to_cv2_image_sliced(message)

    world_coordinates = []
    for image_coordinate in image_coordinates:
        depth_value = depth_image[image_coordinate.y, image_coordinate.x]
        x, y, z = rs2.rs2_deproject_pixel_to_point(
            intrinsics, [image_coordinate.x, image_coordinate.y], depth_value
        )
        world_coordinates.append(Point(x=x, y=y, z=z))

    return world_coordinates


def get_centroid_image_coordinates(mask: np.ndarray) -> ImageCoordinate:
    """Get image coordinate of centroid of object in mask."""
    moments = cv2.moments(mask)
    centroid_x = int(moments["m10"] / moments["m00"])
    centroid_y = int(moments["m01"] / moments["m00"])
    return ImageCoordinate(x=centroid_x, y=centroid_y)


def main(args: str = None) -> None:
    rclpy.init(args=args)

    try:
        detection_service = DetectionService(is_object_of_interest=is_brick)
        rclpy.spin(detection_service)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        detection_service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
