# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import colorsys
from dataclasses import dataclass
from math import pi

import cv2
import numpy as np
import pyrealsense2 as rs2
from rcdt_utilities.geometry import (
    BoundingBox,
    Point2D,
    Point2DList,
    Point3D,
    Pose,
    Quaternion,
)
from rclpy import logging
from typing_extensions import Self

from rcdt_detection.image_manipulation import three_to_single_channel

logger = logging.get_logger(__name__)


@dataclass
class ImageUtils:
    """Utility class for depth image manipulation.

    This class provides methods to work with depth images, including checking point clearance,
    calculating average depth, and checking if points are inside the image dimensions.

    Attributes:
        image (np.ndarray): The depth image in millimeters, where each pixel value represents the depth in millimeters.
    """

    image: np.ndarray

    def average_depth(self, points: Point2DList) -> float:
        """Return the average depth value.

        Args:
            points (Point2DList): A list of 2D points for which the average depth is calculated.

        Returns:
            float: The average depth value in meters.
        """
        return np.mean([self.depth_value(point) for point in points.points])

    def is_point_inside(self, point: Point2D) -> bool:
        """Return if this point falls inside given image dimensions.

        Args:
            point (Point2D): The 2D point to check.

        Returns:
            bool: True if the point is inside the image dimensions, False otherwise.
        """
        rows, cols = self.image.shape
        x, y = point.as_tuple
        return 0 <= x < cols and 0 <= y < rows

    def is_pointlist_inside(self, points: Point2DList) -> bool:
        """Test if all points in group fall inside given image.

        Args:
            points (Point2DList): A list of 2D points to check.

        Returns:
            bool: True if all points are inside the image dimensions, False otherwise.
        """
        return all(self.is_point_inside(point) for point in points.points)

    def is_point_clearance(
        self, point: Point2D, object_depth: float, min_depth: float = 0.04
    ) -> bool:
        """Test if area arround point is deeper than object_depth, and is at least min_depth relative to object_depth.

        Args:
            point (Point2D): The 2D point to check.
            object_depth (float): The depth of the object in meters.
            min_depth (float): The minimum depth difference required from the object depth.

        Returns:
            bool: True if the area around the point is deeper than object_depth and at least min_depth deeper, False otherwise.
        """
        point_group = point.surrounding_points()
        if not self.is_pointlist_inside(point_group):
            return False
        average_depth = self.average_depth(point_group)
        depth_difference = average_depth - object_depth
        return depth_difference > min_depth

    def is_points_clearance(self, object_depth: float, points: Point2DList) -> bool:
        """Test if all corresponding points in depth_image are deeper than object_depth.

        Args:
            object_depth (float): The depth of the object in meters.
            points (Point2DList): A list of 2D points to check.

        Returns:
            bool: True if all points are clear, False otherwise.
        """
        return all(
            self.is_point_clearance(point, object_depth) for point in points.points
        )

    def depth_value(self, point: Point2D) -> float:
        """Return the depth value in meters.

        Args:
            point (Point2D): The 2D point for which the depth value is retrieved.

        Returns:
            float: The depth value in meters.
        """
        return self.image[int(point.y), int(point.x)] / 1000


class MaskProperties:
    """Class to hold properties of a mask and its corresponding depth image.

    This class provides methods to analyze the mask, such as finding contours, bounding boxes,
    centroids, average hue, average depth, and suitable pickup points.

    Atributes:
        mask (np.ndarray): The mask image, typically a 3-channel RGB image.
        depth_image (np.ndarray): The depth image in millimeters, where each pixel value represents the depth in millimeters.
        intrinsics (rs2.intrinsics): The camera intrinsics for depth image.
    """

    def __init__(
        self, mask: np.ndarray, depth_image: np.ndarray, intrinsics: rs2.intrinsics
    ):
        """Initialize the MaskProperties with a mask, depth image, and camera intrinsics.

        Args:
            mask (np.ndarray): The mask image, typically a 3-channel RGB image.
            depth_image (np.ndarray): The depth image in millimeters, where each pixel value represents the depth in millimeters.
            intrinsics (rs2.intrinsics): The camera intrinsics for depth image.
        """
        self.mask = mask
        self.depth_image = depth_image
        self.depth_image_utils = ImageUtils(depth_image)
        self.intrinsics = intrinsics
        self.single_channel = three_to_single_channel(mask)

    @property
    def rows(self) -> int:
        """Returns the number of rows in the depth image."""
        return self.depth_image.shape[0]

    @property
    def cols(self) -> int:
        """Returns the number of columns in the depth image."""
        return self.depth_image.shape[1]

    @property
    def contour(self) -> list[list[np.ndarray]]:
        """Returns the main contour, so that tiny contours due to noise are eliminated.

        The main contour is the largest contour found in the single-channel mask.

        Returns:
            list[list[np.ndarray]]: The largest contour found in the single-channel mask.
        """
        contours, _ = cv2.findContours(
            self.single_channel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        countour_areas = [cv2.contourArea(contour) for contour in contours]
        return contours[countour_areas.index(max(countour_areas))]

    @property
    def bounding_box(self) -> BoundingBox:
        """Returns the bounding box of the main contour.

        The bounding box is calculated using the minimum area rectangle that can enclose the contour.

        Returns:
            BoundingBox: The bounding box of the main contour, represented as a BoundingBox object.
        """
        (x, y), (w, h), angle_deg = cv2.minAreaRect(self.contour)
        return BoundingBox(x, y, w, h, angle_deg)

    @property
    def centroid(self) -> Point2D:
        """Returns the centroid of the mask.

        The centroid is calculated using image moments, which provide the center of mass of the mask.

        Returns:
            Point2D: The centroid of the mask, represented as a Point2D object.
        """
        moments = cv2.moments(self.single_channel)
        x = float(moments["m10"] / moments["m00"])
        y = float(moments["m01"] / moments["m00"])
        return Point2D(x, y)

    @property
    def avg_hue(self) -> float:
        """Returns the average hue of the mask.

        The average hue is calculated by converting the average RGB values of the mask to HSV and extracting the hue component.

        Returns:
            float: The average hue of the mask in degrees, ranging from 0 to 360.
        """
        b_mean, g_mean, r_mean = np.nanmean(
            np.where(self.mask == 0.0, np.nan, self.mask), axis=(0, 1)
        )
        hue_mean, *_ = colorsys.rgb_to_hsv(r_mean, g_mean, b_mean)
        return hue_mean * 360

    @property
    def average_depth(self) -> float:
        """Returns the average depth value of the mask in meters.

        The average depth is calculated using the mean of the depth values in the depth image, masked by the single-channel mask.

        Returns:
            float: The average depth value of the mask in meters.
        """
        average_depth_mm = cv2.mean(self.depth_image, mask=self.single_channel)[0]
        return average_depth_mm / 1000

    @property
    def mode_depth(self) -> float:
        """Returns the mode depth value of the mask.

        The mode depth is calculated by finding the most common depth value in the masked depth image.

        Returns:
            float: The mode depth value of the mask in millimeters.
        """
        masked_depth_values = self.depth_image[self.single_channel > 0]
        return np.bincount(masked_depth_values).argmax()

    def refined_mask(self) -> Self:
        """Returns a refined mask based on the mode depth value.

        The refined mask is created by applying a condition that keeps only the pixels within a certain range around the mode depth value.
        The range is defined as mode_depth ± delta, where delta is set to 30 mm.

        Returns:
            MaskProperties: A new MaskProperties object with the refined mask, depth image, and intrinsics.
        """
        delta = 30
        min_depth = self.mode_depth - delta
        max_depth = self.mode_depth + delta
        condition = (self.depth_image >= min_depth) & (self.depth_image <= max_depth)
        reduced_mask: np.ndarray = self.mask * condition[:, :, np.newaxis]
        return MaskProperties(
            mask=reduced_mask, depth_image=self.depth_image, intrinsics=self.intrinsics
        )

    @property
    def pickup_points(self) -> tuple[Point2DList, Point2DList]:
        """Returns suitable and non-suitable pickup points based on the bounding box's long sides offset.

        The suitable pickup points are those that have sufficient clearance from the object depth,
        while the non-suitable points are those that do not meet the clearance criteria.

        Returns:
            tuple[Point2DList, Point2DList]: A tuple containing two Point2DList objects:
                - suitable: Points that are clear for pickup.
                - non_suitable: Points that are not clear for pickup.
        """
        point_pairs = self.bounding_box.long_sides_offset.point_pairs()
        clearance = np.array(
            [
                self.depth_image_utils.is_points_clearance(self.average_depth, pair)
                for pair in point_pairs
            ]
        )

        centers = np.array([point_pairs[n].mean for n in range(len(point_pairs))])
        suitable = Point2DList(centers[clearance])
        non_suitable = Point2DList(centers[~clearance])
        return suitable, non_suitable

    @property
    def chosen_pickup_point(self) -> Point2D | None:
        """Returns a suitable pickup point based on the available points.

        The chosen pickup point is the middle point of the suitable pickup points.
        If there are no suitable points, it returns None.

        Returns:
            Point2D | None: The chosen pickup point as a Point2D object, or None if no suitable points are available.
        """
        suitable, non_suitable = self.pickup_points

        if len(suitable.points) == 0:
            return None
        return suitable.points[len(suitable.points) // 2]

    @property
    def filter_visualization(self) -> np.ndarray:
        """Returns a visualization of the mask with the bounding box, long sides offset, and pickup points.

        The visualization includes:
            - The bounding box corners connected by lines.
            - The long sides offset represented by magenta lines.
            - Suitable pickup points marked in red.
            - Non-suitable pickup points marked in blue.
        If there are no suitable pickup points, the chosen pickup point is marked in green.

        Returns:
            np.ndarray: The visualization image with the mask, bounding box, long sides offset, and pickup points.
        """
        image = self.mask.copy()

        for i in range(len(self.bounding_box.corners)):
            cv2.line(
                image,
                self.bounding_box.corners[i - 1].as_int_tuple,
                self.bounding_box.corners[i].as_int_tuple,
                (255, 255, 0),
                1,
            )
        cv2.line(
            image,
            self.bounding_box.long_sides_offset.side1.p1.as_int_tuple,
            self.bounding_box.long_sides_offset.side1.p2.as_int_tuple,
            (255, 0, 255),
            1,
        )
        cv2.line(
            image,
            self.bounding_box.long_sides_offset.side2.p1.as_int_tuple,
            self.bounding_box.long_sides_offset.side2.p2.as_int_tuple,
            (255, 0, 255),
            1,
        )
        suitable, non_suitable = self.pickup_points
        for s_point in suitable.points:
            cv2.circle(image, s_point.as_int_tuple, 3, (0, 0, 255), -1)
        for ns_point in non_suitable.points:
            cv2.circle(image, ns_point.as_int_tuple, 3, (255, 0, 0), -1)

        point = self.chosen_pickup_point
        if point is not None:
            cv2.circle(image, point.as_int_tuple, 3, (0, 255, 0), -1)

        for point_pair in self.bounding_box.long_sides_offset.point_pairs():
            cv2.circle(image, point_pair.points[0].as_int_tuple, 3, (255, 255, 255), -1)
            cv2.circle(image, point_pair.points[1].as_int_tuple, 3, (255, 255, 255), -1)
        return image

    def point_2d_to_3d(self, point: Point2D) -> Point3D:
        """Convert a 2D point in the mask to a 3D point using the depth image and camera intrinsics.

        Args:
            point (Point2D): The 2D point to convert.

        Returns:
            Point3D: The corresponding 3D point in the camera coordinate system.
        """
        x, y, z = rs2.rs2_deproject_pixel_to_point(
            self.intrinsics, point.as_array, self.depth_image_utils.depth_value(point)
        )
        return Point3D(x, y, z)

    def point_2d_to_pose(self, point_2d: Point2D) -> Pose:
        """Convert a 2D point in the mask to a Pose object.

        The Pose object is created using the 3D point corresponding to the 2D point and the bounding box angle.

        Args:
            point_2d (Point2D): The 2D point to convert.

        Returns:
            Pose: The corresponding Pose object, which includes the 3D point and the bounding box angle.
        """
        point_3d = self.point_2d_to_3d(point_2d)
        # make sure bounding box angle is in the range of 0 to pi radians
        bounding_box_angle = (
            self.bounding_box.long_sides_offset.side1.angle() % pi
        ) + pi
        return Pose(point_3d, Quaternion.from_eulerangles(0.0, 0.0, bounding_box_angle))
