#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rcdt_detection.mask_properties import MaskProperties
from rcdt_messages.srv import GetMaskProperties as Srv
from rcdt_utilities.cv_utils import camera_info_to_intrinsics, ros_image_to_cv2_image
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging
from rclpy.node import Node
from vision_msgs.msg import Point2D

ros_logger = logging.get_logger(__name__)


class GetMaskProperties(Node):
    """Node to get properties of a mask from a depth image and camera info.

    This node provides a service that takes a mask, depth image, and camera info,
    and returns properties such as contour, bounding box, centroid, and average hue.

    Attributes:
        node_name (str): The name of the node.
        service_name (str): The name of the service provided by this node.
    """

    def __init__(self) -> None:
        """Initialize the GetMaskProperties node."""
        super().__init__("get_mask_properties")
        self.create_service(Srv, "/get_mask_properties", self.callback)

    @staticmethod
    def callback(request: Srv.Request, response: Srv.Response) -> Srv.Response:
        """Callback function to handle the service request.

        Args:
            request (Srv.Request): The request containing the mask, depth image, and camera info.
            response (Srv.Response): The response to be filled with mask properties.

        Returns:
            Srv.Response: The response containing the properties of the mask.
        """
        mask = ros_image_to_cv2_image(request.mask)
        depth_image = ros_image_to_cv2_image(request.depth_image)
        intrinsics = camera_info_to_intrinsics(request.camera_info)

        mask_properties = MaskProperties(mask, depth_image, intrinsics)

        for point in mask_properties.contour:
            point_2d = Point2D(x=float(point[0, 0]), y=float(point[0, 1]))
            response.contour.append(point_2d)
        for point in mask_properties.bounding_box.corners:
            point_2d = Point2D(x=float(point.x), y=float(point.y))
            response.box_contour.append(point_2d)
        point = mask_properties.centroid
        response.centroid = Point2D(x=float(point.x), y=float(point.y))
        response.avg_hue = float(mask_properties.avg_hue)

        response.success = True
        return response


def main(args: str = None) -> None:
    """Initialize the ROS 2 node and spin it.

    Args:
        args (str, optional): Command line arguments. Defaults to None.
    """
    rclpy.init(args=args)
    node = GetMaskProperties()
    spin_node(node)


if __name__ == "__main__":
    main()
