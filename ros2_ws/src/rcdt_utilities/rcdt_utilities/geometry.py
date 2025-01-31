# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
import cv2
import numpy as np
from math import atan2
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import (
    Point as ros_Point,
    Quaternion as ros_Quaternion,
    Pose as ros_Pose,
)


@dataclass
class Point2D:
    x: float
    y: float

    @property
    def as_tuple(self) -> tuple:
        """Return point as tuple."""
        return self.x, self.y

    @property
    def as_list(self) -> list:
        return [self.x, self.y]

    def depth_value(self, depth_image: np.ndarray) -> float:
        """Return the depth value in meters."""
        return depth_image[self.y, self.x] / 1000

    def surrounding_points(self, grid_size: np.ndarray = 5) -> "Point2DList":
        """Return a dense list of 2D points around self with distance of up to grid_size."""
        surrounding_points = []
        for x_pos in range(self.x - grid_size, self.x + grid_size):
            for y_pos in range(self.y - grid_size, self.y + grid_size):
                surrounding_points.append(Point2D([x_pos, y_pos]))
        return Point2DList(surrounding_points)

    # def is_inside(self, image: np.ndarray) -> bool:
    #     """Return if this point falls inside given image dimensions."""
    #     rows, cols = image.shape
    #     x, y = self.as_tuple
    #     return 0 <= x < cols and 0 <= y < rows

    # def is_clearance(
    #     self, depth_image: np.ndarray, object_depth: float, min_depth: float = 0.05
    # ) -> bool:
    #     """Test if corresponding point in depth_image is deeper than min_depth."""
    #     point_group = self.surrounding_points()
    #     if not point_group.is_all_inside(depth_image):
    #         return False

    #     average_depth = point_group.average_depth(depth_image)
    #     depth_difference = average_depth - object_depth
    #     return depth_difference > min_depth


@dataclass
class Point3D:
    x: float
    y: float
    z: float

    @property
    def as_ros_point(self) -> ros_Point:
        return ros_Point(x=self.x, y=self.y, z=self.z)


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    @property
    def as_tuple(self) -> tuple:
        return self.x, self.y, self.z, self.w

    @staticmethod
    def from_eulerangles(x: float, y: float, z: float) -> "Quaternion":
        return Quaternion(*quaternion_from_euler(x, y, z))

    @property
    def as_ros_quaternion(self) -> ros_Quaternion:
        return ros_Quaternion(x=self.x, y=self.y, z=self.z, w=self.w)


@dataclass
class Pose:
    position: Point3D
    orientation: Quaternion

    def as_ros_pose(self) -> ros_Pose:
        return ros_Pose(
            position=self.position.as_ros_point,
            orientation=self.orientation.as_ros_quaternion,
        )


@dataclass
class Point2DList:
    points: list[Point2D]

    @property
    def mean(self) -> Point2D:
        return Point2D(np.mean([point.array for point in self.points], axis=0))

    ### def average_depth(self, depth_image: np.ndarray) -> float:
    #     """Return the average depth value in meters."""
    #     return np.mean([point.depth_value(depth_image) for point in self.points])

    #### def is_all_inside(self, image: np.ndarray) -> bool:
    #     """Return if all points in group fall inside given image."""
    #     return all(point.is_inside(image) for point in self.points)

    #### def is_clearance(self, depth_image: np.ndarray, object_depth: float) -> bool:
    #     """Test if all corresponding points in depth_image are deeper than object_depth."""
    #     return all(
    #         point.is_clearance(depth_image, object_depth) for point in self.points
    #     )


@dataclass
class Line:
    p1: Point2D
    p2: Point2D

    @property
    def length(self) -> float:
        return np.linalg.norm(self.p1.as_list() - self.p2.as_list())

    @property
    def flipped(self) -> "Line":
        return Line(self.p2, self.p1)

    def points_along(self, n: int) -> list[Point2D]:
        points = np.linspace(self.p1.array, self.p2.array, n + 2, dtype=int)
        points = points[1:-1]  # remove edges
        return [Point2D(*point) for point in points]

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
    angle_deg: float

    @property
    def ordered_values(self) -> tuple[float]:
        return (self.x, self.y, self.w, self.h, self.angle_deg)

    @property
    def center(self) -> Point2D:
        return Point2D([self.x, self.y])

    @property
    def corners(self) -> list[Point2D]:
        x, y, w, h, angle_deg = self.ordered_values
        return [Point2D(point) for point in cv2.boxPoints(((x, y), (w, h), angle_deg))]

    @property
    def long_sides_offset(self) -> SidePair:
        offset_factor: float = 1.4
        if self.h > self.w:
            self.w *= offset_factor
        else:
            self.h *= offset_factor
        corners = cv2.boxPoints(((self.x, self.y), (self.w, self.h), self.angle_deg))
        points = [Point2D(*corner) for corner in corners]
        sides = [Line(points[n], points[n - 1]) for n in range(len(corners))]
        sides.sort(key=lambda side: side.length)
        side1, side2 = sides[2:]
        return SidePair(side1, side2.flipped)
