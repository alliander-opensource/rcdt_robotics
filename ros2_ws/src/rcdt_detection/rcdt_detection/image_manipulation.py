# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import cv2
from numpy import ndarray, uint8
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import floor, ceil

cv_bridge = CvBridge()


def ros_image_to_cv2_image(
    image_message: Image, desired_encoding: str = "passthrough"
) -> ndarray:
    """Convert ROS image message to cv2 image."""
    return cv_bridge.imgmsg_to_cv2(image_message, desired_encoding=desired_encoding)


def cv2_image_to_ros_image(image: ndarray, encoding: str) -> ndarray:
    """Convert cv2 image message to ROS image."""
    return cv_bridge.cv2_to_imgmsg(image, encoding=encoding)


def slice_image_to_stride(image: ndarray, stride: int = 32) -> ndarray:
    """Slice image to confirm to a given stride length"""
    rows = image.shape[0]
    cols = image.shape[1]
    row_dist = rows % stride
    col_dist = cols % stride
    return image[
        slice(floor(row_dist / 2), rows - ceil(row_dist / 2)),
        slice(floor(col_dist / 2), cols - ceil(col_dist / 2)),
    ]


def ros_image_to_cv2_image_sliced(
    image_message: Image, desired_encoding: str = "passthrough", stride: int = 32
) -> ndarray:
    """slice an image so that its dimensions are a multiple of stride"""
    return slice_image_to_stride(
        ros_image_to_cv2_image(image_message, desired_encoding), stride
    )


def segmentation_mask_to_binary_mask(mask: torch.Tensor) -> ndarray:
    """Convert given mask to np.ndarray with range [0, 255], dtype=uint8, and dimensions [height, width, channels]."""
    binary_mask = mask.data.cpu().numpy().astype(uint8)
    binary_mask = binary_mask * 255
    binary_mask = binary_mask.transpose(1, 2, 0)
    return binary_mask


def single_to_three_channel(image: ndarray) -> ndarray:
    """Convert given single-channel image to three-channel image."""
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
