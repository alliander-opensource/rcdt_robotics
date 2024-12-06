# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import cv2
from numpy import ndarray, uint8
import torch
from sensor_msgs.msg import Image
from rcdt_utilities import cv_utils
from math import floor, ceil


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
    """Slice an image so that its dimensions are a multiple of stride"""
    return slice_image_to_stride(
        cv_utils.ros_image_to_cv2_image(image_message, desired_encoding), stride
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


def three_to_single_channel(image: ndarray) -> ndarray:
    """Convert given three-channel image to single-channel image."""
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
