# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from numpy import ndarray

cv_bridge = CvBridge()


def ros_image_to_cv2_image(
    image_message: Image, desired_encoding: str = "passthrough"
) -> ndarray:
    """Convert ROS image message to cv2 image."""
    return cv_bridge.imgmsg_to_cv2(image_message, desired_encoding=desired_encoding)


def cv2_image_to_ros_image(image: ndarray, encoding: str = "passthrough") -> Image:
    """Convert cv2 image message to ROS image."""
    return cv_bridge.cv2_to_imgmsg(image, encoding=encoding)
