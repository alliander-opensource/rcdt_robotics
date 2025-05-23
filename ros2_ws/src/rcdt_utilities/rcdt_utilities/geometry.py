# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from math import atan2

import cv2
import numpy as np
from geometry_msgs.msg import Point as ros_Point
from geometry_msgs.msg import Pose as ros_Pose
from geometry_msgs.msg import Quaternion as ros_Quaternion
from tf_transformations import quaternion_from_euler


@dataclass
class Point2D:
    x: float
    y: float

    @property
    def as_tuple(self) -> tuple:
        """Return point as tuple."""
        return self.x, self.y

    @property
    def as_int_tuple(self) -> tuple:
        return int(self.x), int(self.y)

    @property
    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y])

    def surrounding_points(self, grid_size: int = 5) -> "Point2DList":
        """Return a dense list of 2D points around self with distance of up to grid_size."""
        surrounding_points = []
        for x_pos in range(self.x - grid_size, self.x + grid_size):
            for y_pos in range(self.y - grid_size, self.y + grid_size):
                surrounding_points.append(Point2D(x_pos, y_pos))
        return Point2DList(surrounding_points)


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
        x, y, z, w = quaternion_from_euler(x, y, z)
        return Quaternion(x, y, z, w)

    @staticmethod
    def from_eulerangles_deg(x: float, y: float, z: float) -> "Quaternion":
        x, y, z, w = quaternion_from_euler(np.deg2rad(x), np.deg2rad(y), np.deg2rad(z))
        return Quaternion(x, y, z, w)

    @property
    def as_ros_quaternion(self) -> ros_Quaternion:
        return ros_Quaternion(x=self.x, y=self.y, z=self.z, w=self.w)


@dataclass
class Pose:
    position: Point3D
    orientation: Quaternion

    @property
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
        x, y = np.mean([point.as_array for point in self.points], axis=0)
        return Point2D(x, y)


@dataclass
class Line:
    p1: Point2D
    p2: Point2D

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.p1.as_array - self.p2.as_array))

    @property
    def flipped(self) -> "Line":
        return Line(self.p2, self.p1)

    def points_along(self, n: int) -> list[Point2D]:
        points = np.linspace(self.p1.as_array, self.p2.as_array, n + 2, dtype=int)
        points = points[1:-1]  # remove edges
        return [Point2D(point[0], point[1]) for point in points]

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
        return Point2D(self.x, self.y)

    @property
    def corners(self) -> list[Point2D]:
        x, y, w, h, angle_deg = self.ordered_values
        return [Point2D(*point) for point in cv2.boxPoints(((x, y), (w, h), angle_deg))]

    @property
    def long_sides_offset(self) -> SidePair:
        offset_factor: float = 1.6
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
