# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


from dataclasses import dataclass
import cv2
import colorsys
import numpy as np
import pyrealsense2 as rs2
from math import atan2
from rcdt_detection.image_manipulation import three_to_single_channel
from math import pi

from rclpy.logging import get_logger

logger = get_logger(__name__)


@dataclass
class Point2D:
    array: np.ndarray

    @property
    def x(self) -> int:
        return int(self.array[0])

    @property
    def y(self) -> int:
        return int(self.array[1])

    @property
    def as_tuple(self) -> tuple:
        """Return point as tuple."""
        return self.x, self.y

    def depth_value(self, depth_image: np.ndarray) -> float:
        """Return the depth value in meters."""
        return depth_image[self.y, self.x] / 1000

    def surrounding_points(self, grid_size: np.ndarray = 5) -> "Point2DList":
        """Return a dense list of 2D points around self with distance of up to d."""
        surrounding_points = []
        for x_pos in range(self.x - grid_size, self.x + grid_size):
            for y_pos in range(self.y - grid_size, self.y + grid_size):
                surrounding_points.append(Point2D([x_pos, y_pos]))
        return Point2DList(surrounding_points)

    def is_inside(self, image: np.ndarray) -> bool:
        """Return if this point falls inside given image dimensions."""
        rows, cols = image.shape
        x, y = self.as_tuple
        return 0 <= x < cols and 0 <= y < rows

    def is_clearance(
        self, depth_image: np.ndarray, object_depth: float, min_depth: float = 0.05
    ) -> bool:
        """test if corresponding point in depth_image is deeper than min_depth."""
        point_group = self.surrounding_points()
        if not point_group.is_all_inside(depth_image):
            return False

        average_depth = point_group.average_depth(depth_image)
        depth_difference = average_depth - object_depth
        return depth_difference > min_depth


@dataclass
class Point3D:
    array: np.ndarray

    @property
    def x(self) -> int:
        return float(self.array[0])

    @property
    def y(self) -> int:
        return float(self.array[1])

    @property
    def z(self) -> int:
        return float(self.array[2])


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    @property
    def as_tuple(self) -> tuple:
        return self.x, self.y, self.z, self.w


@dataclass
class Point2DList:
    points: list[Point2D]

    @property
    def mean(self) -> Point2D:
        return Point2D(np.mean([point.array for point in self.points], axis=0))

    def average_depth(self, depth_image: np.ndarray) -> float:
        """Return the average depth value in meters."""
        return np.mean([point.depth_value(depth_image) for point in self.points])

    def is_all_inside(self, image: np.ndarray) -> bool:
        """Return if all points in group fall inside given image."""
        return all(point.is_inside(image) for point in self.points)

    def is_clearance(self, depth_image: np.ndarray, object_depth: float) -> bool:
        """test if all corresponding points in depth_image are deeper than min_depth."""
        return all(
            point.is_clearance(depth_image, object_depth) for point in self.points
        )


@dataclass
class Line:
    p1: Point2D
    p2: Point2D

    @property
    def length(self) -> float:
        return np.linalg.norm(self.p1.array - self.p2.array)

    @property
    def flipped(self) -> "Line":
        return Line(self.p2, self.p1)

    def points_along(self, n: int) -> list[Point2D]:
        points = np.linspace(self.p1.array, self.p2.array, n + 2, dtype=int)
        points = points[1:-1]  # remove edges
        return [Point2D(point) for point in points]

    def angle(self) -> float:
        dx = self.p2.x - self.p1.x
        dy = self.p2.y - self.p1.y
        return atan2(dy, dx)


@dataclass
class SidePair:
    side1: Line
    side2: Line

    def point_pairs(self, n: int = 5) -> list[Point2DList]:
        points1 = self.side1.points_along(n)
        points2 = self.side2.points_along(n)
        return [Point2DList([points1[i], points2[i]]) for i in range(n)]


@dataclass
class BoundingBox:
    x: float
    y: float
    w: float
    h: float
    angle: float

    def long_sides_offset(self) -> SidePair:
        offset_factor: float = 1.4
        if self.h > self.w:
            self.w *= offset_factor
        else:
            self.h *= offset_factor
        corners = cv2.boxPoints(((self.x, self.y), (self.w, self.h), self.angle))
        points = [Point2D(corner) for corner in corners]
        sides = [Line(points[n], points[n - 1]) for n in range(len(corners))]
        sides.sort(key=lambda side: side.length)
        side1, side2 = sides[2:]
        return SidePair(side1, side2.flipped)

    @property
    def ordered_values(self) -> tuple[float]:
        return (self.x, self.y, self.w, self.h, self.angle)

    @property
    def center(self) -> Point2D:
        return Point2D([self.x, self.y])

    @property
    def corners(self) -> list[Point2D]:
        x, y, w, h, angle = self.ordered_values
        return [Point2D(point) for point in cv2.boxPoints(((x, y), (w, h), angle))]


class MaskProperties:
    def __init__(
        self, mask: np.ndarray, depth_image: np.ndarray, intrinsics: rs2.intrinsics
    ):
        self.mask = mask
        self.depth_image = depth_image
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
        (x, y), (w, h), angle = cv2.minAreaRect(self.contour)
        return BoundingBox(x, y, w, h, angle)

    @property
    def centroid(self) -> Point2D:
        """Returns the centroid of the mask."""
        moments = cv2.moments(self.single_channel)
        x = float(moments["m10"] / moments["m00"])
        y = float(moments["m01"] / moments["m00"])
        return Point2D([x, y])

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
    def pickup_points(self) -> tuple[Point2DList, Point2DList]:
        long_sides = self.bounding_box.long_sides_offset()
        point_pairs = long_sides.point_pairs()
        clearance = np.array(
            [
                pair.is_clearance(self.depth_image, self.average_depth)
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
                self.bounding_box.corners[i - 1].as_tuple,
                self.bounding_box.corners[i].as_tuple,
                (255, 255, 0),
                1,
            )
        cv2.line(
            image,
            self.bounding_box.long_sides_offset().side1.p1.as_tuple,
            self.bounding_box.long_sides_offset().side1.p2.as_tuple,
            (255, 0, 255),
            1,
        )
        cv2.line(
            image,
            self.bounding_box.long_sides_offset().side2.p1.as_tuple,
            self.bounding_box.long_sides_offset().side2.p2.as_tuple,
            (255, 0, 255),
            1,
        )
        suitable, non_suitable = self.pickup_points
        for point in suitable.points:
            cv2.circle(image, point.as_tuple, 3, (0, 0, 255), -1)
        for point in non_suitable.points:
            cv2.circle(image, point.as_tuple, 3, (255, 0, 0), -1)

        point = self.chosen_pickup_point
        if point is not None:
            cv2.circle(image, point.as_tuple, 3, (0, 255, 0), -1)

        for point_pair in self.bounding_box.long_sides_offset().point_pairs():
            cv2.circle(image, point_pair.points[0].as_tuple, 3, (255, 255, 255), -1)
            cv2.circle(image, point_pair.points[1].as_tuple, 3, (255, 255, 255), -1)
        return image

    def point_2d_to_3d(self, point: Point2D) -> Point3D:
        x, y, z = rs2.rs2_deproject_pixel_to_point(
            self.intrinsics, point.array, point.depth_value(self.depth_image)
        )
        return Point3D(np.array([x, y, z]))

    def point_2d_to_pose(self, point_2d: Point2D) -> tuple:
        point_3d = self.point_2d_to_3d(point_2d)
        mask_angle = self.bounding_box.long_sides_offset().side1.angle()# + pi
        return point_3d, (0.0, 0.0, mask_angle)
