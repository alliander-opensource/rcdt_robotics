from dataclasses import dataclass
import cv2
import numpy as np
import pyrealsense2 as rs2
from rcdt_detection.image_manipulation import three_to_single_channel

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
    def tuple(self) -> tuple:
        """Return point as tuple."""
        return self.x, self.y

    def depth_value(self, depth_image: np.ndarray) -> float:
        """Return the depth value in meters."""
        return depth_image[self.y, self.x] / 1000

    def surrounding_points(self, d: np.ndarray) -> "Point2DGroup":
        """Return a list of 2D points around self with distance d."""
        x, y = self.tuple
        points = np.mgrid[x - d : x + d, y - d : y + d].reshape(2, -1).T
        return Point2DGroup([Point2D(point) for point in points])

    def is_valid(self, image: np.ndarray) -> bool:
        """Return whether point is in given image."""
        rows, cols = image.shape
        x, y = self.tuple
        return 0 <= x < cols and 0 <= y < rows

    def is_space(self, depth_image: np.ndarray, object_depth: float) -> bool:
        """Return wether there is space."""
        OFFSET_POINTS = 5
        MINIMUM_DEPTH_DIFFERENCE = 0.05

        point_group = self.surrounding_points(OFFSET_POINTS)
        if not point_group.is_valid(depth_image):
            return False

        depth = point_group.average_depth_value(depth_image)
        depth_difference = depth - object_depth
        return depth_difference > MINIMUM_DEPTH_DIFFERENCE


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
class Point2DGroup:
    points: list[Point2D]

    @property
    def mean(self) -> Point2D:
        return Point2D(np.mean([point.array for point in self.points], axis=0))

    def average_depth_value(self, depth_image: np.ndarray) -> float:
        """Return the average depth value in meters."""
        return np.mean([point.depth_value(depth_image) for point in self.points])

    def is_valid(self, image: np.ndarray) -> bool:
        """Return whether all points in group are in given image."""
        return all(point.is_valid(image) for point in self.points)

    def is_space(self, depth_image: np.ndarray, object_depth: float) -> bool:
        """Return whether there is space for all points in the group."""
        return all(point.is_space(depth_image, object_depth) for point in self.points)


@dataclass
class Side:
    p1: Point2D
    p2: Point2D

    @property
    def length(self) -> float:
        return np.linalg.norm(self.p1.array - self.p2.array)

    @property
    def flipped(self) -> "Side":
        return Side(self.p2, self.p1)

    def points(self, n: int) -> list[Point2D]:
        points = np.linspace(self.p1.array, self.p2.array, n + 2, dtype=int)
        points = points[1:-1]  # remove edges
        return [Point2D(point) for point in points]


@dataclass
class SideSet:
    side1: Side
    side2: Side

    def point_groups(self, n: int) -> list[Point2DGroup]:
        points1 = self.side1.points(n)
        points2 = self.side2.points(n)
        return [Point2DGroup([points1[i], points2[i]]) for i in range(n)]


@dataclass
class BoundingBox:
    x: float
    y: float
    w: float
    h: float
    angle: float

    def long_sides(self, offset_factor: float) -> SideSet:
        x, y, w, h, angle = self.ordered_values
        h *= offset_factor

        corners = cv2.boxPoints(((x, y), (w, h), angle))
        points = [Point2D(corner) for corner in corners]
        sides = [Side(points[n], points[n - 1]) for n in range(len(corners))]

        sides.sort(key=lambda side: side.length)
        side1, side2 = sides[2:]
        return SideSet(side1, side2.flipped)

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
        self.img_depth = depth_image
        self.intrinsics = intrinsics
        self.single_channel = three_to_single_channel(mask)

    @property
    def rows(self) -> int:
        return self.img_depth.shape[0]

    @property
    def cols(self) -> int:
        return self.img_depth.shape[1]

    @property
    def contour(self) -> list[list[np.ndarray]]:
        contours, _ = cv2.findContours(
            self.single_channel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        countour_areas = [cv2.contourArea(contour) for contour in contours]
        return contours[countour_areas.index(max(countour_areas))]

    @property
    def bounding_box(self) -> BoundingBox:
        (x, y), (w, h), angle = cv2.minAreaRect(self.contour)
        return BoundingBox(x, y, w, h, angle)

    @property
    def avg_depth(self) -> float:
        """Returns the average depth value of the mask in meters."""
        average_depth_mm = cv2.mean(self.img_depth, mask=self.single_channel)[0]
        return average_depth_mm / 1000

    @property
    def pick_locations(self) -> tuple[Point2DGroup, Point2DGroup]:
        OFFSET_FACTOR = 1.4
        POINTS = 5

        sides = self.bounding_box.long_sides(OFFSET_FACTOR)
        groups = sides.point_groups(POINTS)
        space = np.array(
            [group.is_space(self.img_depth, self.avg_depth) for group in groups]
        )

        centers = np.array([groups[n].mean for n in range(POINTS)])
        suitable = Point2DGroup(centers[space])
        non_suitable = Point2DGroup(centers[~space])

        return suitable, non_suitable

    @property
    def pick_location(self) -> Point2D | None:
        suitable, non_suitable = self.pick_locations

        if len(suitable.points) == 0:
            return None

        middle = suitable.points[len(suitable.points) // 2]
        return middle

    @property
    def filter_visualization(self) -> np.ndarray:
        image = self.mask.copy()

        suitable, non_suitable = self.pick_locations
        for point in suitable.points:
            cv2.circle(image, point.tuple, 3, (0, 0, 255), -1)
        for point in non_suitable.points:
            cv2.circle(image, point.tuple, 3, (255, 0, 0), -1)

        point = self.pick_location
        if point is not None:
            cv2.circle(image, point.tuple, 3, (0, 255, 0), -1)

        return image

    def point_2d_to_3d(self, point: Point2D) -> Point3D:
        x, y, z = rs2.rs2_deproject_pixel_to_point(
            self.intrinsics, point.array, point.depth_value(self.img_depth)
        )
        return Point3D(np.array([x, y, z]))
