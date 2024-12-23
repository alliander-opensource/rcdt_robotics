from dataclasses import dataclass
import cv2
import numpy as np
import pyrealsense2 as rs2
from rcdt_detection.image_manipulation import three_to_single_channel


@dataclass
class BoundingBox:
    x: float
    y: float
    w: float
    h: float
    angle: float

    def ordered_values(self) -> tuple[float]:
        return (self.x, self.y, self.w, self.h, self.angle)


class MaskProperties:
    def __init__(
        self, mask: np.ndarray, depth_image: np.ndarray, intrinsics: rs2.intrinsics
    ):
        self.mask = mask
        self.depth_image = depth_image
        self.intrinsics = intrinsics
        self.single_channel = three_to_single_channel(mask)

    @property
    def contour(self) -> list[list[np.ndarray]]:
        contours, _ = cv2.findContours(
            self.single_channel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        countour_areas = [cv2.contourArea(contour) for contour in contours]
        return contours[countour_areas.index(max(countour_areas))]

    @property
    def bounding_box_2d(self) -> BoundingBox:
        (x, y), (w, h), angle = cv2.minAreaRect(self.contour)
        return BoundingBox(x, y, w, h, angle)

    @property
    def box_contour_2d(self) -> np.ndarray:
        x, y, w, h, angle = self.bounding_box_2d.ordered_values()
        return np.int32(cv2.boxPoints(((x, y), (w, h), angle)))

    @property
    def box_contour_3d(self) -> list[np.ndarray]:
        return np.array(
            [self.point_2d_to_3d(point_2d) for point_2d in self.box_contour_2d]
        )

    @property
    def box_shape_3d(self) -> list[float]:
        sides = []
        points = self.box_contour_3d
        for n in range(len(points)):
            sides.append(np.linalg.norm(points[n] - points[n - 1]))
        length1 = np.mean([sides[0], sides[2]])
        length2 = np.mean([sides[1], sides[3]])
        return [min(length1, length2), max(length1, length2)]

    def point_2d_to_3d(self, point_2d: np.ndarray) -> np.ndarray:
        depth_value = self.depth_image[point_2d[1], point_2d[0]]
        x_mm, y_mm, z_mm = rs2.rs2_deproject_pixel_to_point(
            self.intrinsics, point_2d, depth_value
        )
        x_m, y_m, z_m = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0
        return np.array([x_m, y_m, z_m])
