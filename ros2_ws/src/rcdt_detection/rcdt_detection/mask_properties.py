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

from rcdt_detection.image_manipulation import (
    single_to_three_channel,
    three_to_single_channel,
)

logger = logging.get_logger(__name__)


@dataclass
class ImageUtils:
    image: np.ndarray

    def average_depth(self, points: Point2DList) -> float:
        """Return the average depth value."""
        return np.mean([self.depth_value(point) for point in points.points])

    def is_point_inside(self, point: Point2D) -> bool:
        """Return if this point falls inside given image dimensions."""
        rows, cols = self.image.shape
        x, y = point.as_tuple
        return 0 <= x < cols and 0 <= y < rows

    def is_pointlist_inside(self, points: Point2DList) -> bool:
        """Test if all points in group fall inside given image."""
        return all(self.is_point_inside(point) for point in points.points)

    def is_point_clearance(
        self, point: Point2D, object_depth: float, min_depth: float = 0.04
    ) -> bool:
        """Test if area arround point is deeper than object_depth, and is at least min_depth relative to object_depth."""
        point_group = point.surrounding_points()
        if not self.is_pointlist_inside(point_group):
            return False
        average_depth = self.average_depth(point_group)
        depth_difference = average_depth - object_depth
        return depth_difference > min_depth

    def is_points_clearance(self, object_depth: float, points: Point2DList) -> bool:
        """Test if all corresponding points in depth_image are deeper than object_depth."""
        return all(
            self.is_point_clearance(point, object_depth) for point in points.points
        )

    def depth_value(self, point: Point2D) -> float:
        """Return the depth value in meters."""
        return self.image[int(point.y), int(point.x)] / 1000


class MaskProperties:
    def __init__(
        self, mask: np.ndarray, depth_image: np.ndarray, intrinsics: rs2.intrinsics
    ):
        self.mask = mask
        self.depth_image = depth_image
        self.depth_image_utils = ImageUtils(depth_image)
        self.intrinsics = intrinsics
        self.single_channel = three_to_single_channel(mask)

    @property
    def rows(self) -> int:
        return self.depth_image.shape[0]

    @property
    def cols(self) -> int:
        return self.depth_image.shape[1]

    @property
    def contour(self) -> list[list[np.ndarray]]:
        """Returns the main contour, so that tiny contours due to noise are eliminated."""
        contours, _ = cv2.findContours(
            self.single_channel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        countour_areas = [cv2.contourArea(contour) for contour in contours]
        return contours[countour_areas.index(max(countour_areas))]

    @property
    def bounding_box(self) -> BoundingBox:
        """Returns the bounding box of the main contour."""
        (x, y), (w, h), angle_deg = cv2.minAreaRect(self.contour)
        return BoundingBox(x, y, w, h, angle_deg)

    @property
    def centroid(self) -> Point2D:
        """Returns the centroid of the mask."""
        moments = cv2.moments(self.single_channel)
        x = float(moments["m10"] / moments["m00"])
        y = float(moments["m01"] / moments["m00"])
        return Point2D(x, y)

    @property
    def avg_hue(self) -> float:
        """Returns average hue of the mask."""
        b_mean, g_mean, r_mean = np.nanmean(
            np.where(self.mask == 0.0, np.nan, self.mask), axis=(0, 1)
        )
        hue_mean, *_ = colorsys.rgb_to_hsv(r_mean, g_mean, b_mean)
        return hue_mean * 360

    @property
    def average_depth(self) -> float:
        """Returns the average depth value of the mask in meters."""
        average_depth_mm = cv2.mean(self.depth_image, mask=self.single_channel)[0]
        return average_depth_mm / 1000

    @property
    def mode_depth(self) -> float:
        """return the most common (mode) depth value of the mask."""
        masked_depth_values = self.depth_image[self.single_channel > 0]
        return np.bincount(masked_depth_values).argmax()

    def refined_mask(self) -> Self:
        min_depth = self.mode_depth - 10
        max_depth = self.mode_depth + 10
        condition = (self.depth_image >= min_depth) & (self.depth_image <= max_depth)
        reduced_mask: np.ndarray = self.mask * condition[:, :, np.newaxis]
        return MaskProperties(
            mask=reduced_mask, depth_image=self.depth_image, intrinsics=self.intrinsics
        )

    @property
    def pickup_points(self) -> tuple[Point2DList, Point2DList]:
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
        suitable, non_suitable = self.pickup_points

        if len(suitable.points) == 0:
            return None
        return suitable.points[len(suitable.points) // 2]

    @property
    def filter_visualization(self) -> np.ndarray:
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
        x, y, z = rs2.rs2_deproject_pixel_to_point(
            self.intrinsics, point.as_array, self.depth_image_utils.depth_value(point)
        )
        return Point3D(x, y, z)

    def point_2d_to_pose(self, point_2d: Point2D) -> Pose:
        point_3d = self.point_2d_to_3d(point_2d)
        # make sure bounding box angle is in the range of 0 to pi radians
        bounding_box_angle = (
            self.bounding_box.long_sides_offset.side1.angle() % pi
        ) + pi
        return Pose(point_3d, Quaternion.from_eulerangles(0.0, 0.0, bounding_box_angle))
