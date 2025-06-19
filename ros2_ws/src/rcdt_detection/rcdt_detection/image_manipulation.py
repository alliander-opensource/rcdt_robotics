# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from math import ceil, floor

import cv2
import pyrealsense2 as rs2
import torch
from numpy import ndarray, uint8
from rcdt_utilities import cv_utils
from sensor_msgs.msg import CameraInfo, Image


def camera_info_to_intrinsics(camera_info: CameraInfo) -> rs2.intrinsics:
    """Calculate camera intrinsics for translating image to world coordinates.

    Args:
        camera_info (CameraInfo): The camera info message containing the camera parameters.

    Returns:
        rs2.intrinsics: The calculated camera intrinsics.
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


def slice_image_to_stride(image: ndarray, stride: int = 32) -> ndarray:
    """Slice image to confirm to a given stride length.

    Args:
        image (ndarray): The input image to be sliced.
        stride (int): The stride length to which the image dimensions should conform.

    Returns:
        ndarray: The sliced image with dimensions that are multiples of the stride.
    """
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
    """Slice an image so that its dimensions are a multiple of stride.

    Args:
        image_message (Image): The ROS Image message to be converted.
        desired_encoding (str): The desired encoding for the image.
        stride (int): The stride length to which the image dimensions should conform.

    Returns:
        ndarray: The sliced image with dimensions that are multiples of the stride.
    """
    return slice_image_to_stride(
        cv_utils.ros_image_to_cv2_image(image_message, desired_encoding), stride
    )


def segmentation_mask_to_binary_mask(mask: torch.Tensor) -> ndarray:
    """Convert given mask to np.ndarray with range [0, 255], dtype=uint8, and dimensions [height, width, channels].

    Args:
        mask (torch.Tensor): The input mask tensor, typically with shape [1, channels, height, width].

    Returns:
        ndarray: The binary mask as a numpy array with shape [height, width, channels] and values in the range [0, 255].
    """
    binary_mask = mask.data.cpu().numpy().astype(uint8)
    binary_mask *= 255
    binary_mask = binary_mask.transpose(1, 2, 0)
    return binary_mask


def single_to_three_channel(image: ndarray) -> ndarray:
    """Convert given single-channel image to three-channel image.

    Args:
        image (ndarray): The input single-channel image.

    Returns:
        ndarray: The converted three-channel image.
    """
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def three_to_single_channel(image: ndarray) -> ndarray:
    """Convert given three-channel image to single-channel image.

    Args:
        image (ndarray): The input three-channel image.

    Returns:
        ndarray: The converted single-channel image.
    """
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
