#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from copy import copy

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Transform, TransformStamped
from rcdt_messages.srv import ExpressPoseInOtherFrame, TransformPose
from rcdt_utilities.launch_utils import spin_node
from rclpy import logging, time
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import TransformException  # ty: ignore[unresolved-import]
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

ros_logger = logging.get_logger(__name__)


class ManipulatePose(Node):
    """Node to manipulate poses in different frames and apply transformations.

    Atributes:
        tf_buffer (Buffer): Buffer to store transformations.
        tf_listener (TransformListener): Listener to receive transformations.

    """

    def __init__(self):
        """Initialize the ManipulatePose node."""
        super().__init__("manipulate_pose")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_service(
            ExpressPoseInOtherFrame,
            "/express_pose_in_other_frame",
            self.express_pose_in_other_frame,
        )
        self.create_service(
            TransformPose,
            "/transform_pose",
            self.transform_pose_relative,
        )

    def express_pose_in_other_frame(
        self,
        request: ExpressPoseInOtherFrame.Request,
        response: ExpressPoseInOtherFrame.Response,
    ) -> ExpressPoseInOtherFrame.Response:
        """Express a pose in another frame.

        Args:
            request (ExpressPoseInOtherFrame.Request): The request containing the pose and target frame.
            response (ExpressPoseInOtherFrame.Response): The response to be filled with the transformed pose.

        Returns:
            ExpressPoseInOtherFrame.Response: The response containing the transformed pose and success status.
        """
        source_frame = request.pose.header.frame_id
        target_frame = request.target_frame

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, time.Time()
            )
        except TransformException as ex:
            self.get_logger().error(
                f"Could not transform {target_frame} to {source_frame}: {ex}"
            )
            response.success = False
            return response
        response.pose = do_transform_pose_stamped(request.pose, transform)
        response.success = True
        return response

    @staticmethod
    def transform_pose(
        request: TransformPose.Request, response: TransformPose.Response
    ) -> TransformPose.Response:
        """Transform a pose using a given transformation.

        Args:
            request (TransformPose.Request): The request containing the pose and transformation.
            response (TransformPose.Response): The response to be filled with the transformed pose.

        Returns:
            TransformPose.Response: The response containing the transformed pose and success status.
        """
        response.pose = apply_transform(request.pose, request.transform)
        response.success = True
        return response

    @staticmethod
    def transform_pose_relative(
        request: TransformPose.Request, response: TransformPose.Response
    ) -> TransformPose.Response:
        """Transform a pose relative to its current position using a given transformation.

        Args:
            request (TransformPose.Request): The request containing the pose and transformation.
            response (TransformPose.Response): The response to be filled with the transformed pose.

        Returns:
            TransformPose.Response: The response containing the transformed pose and success status.
        """
        response.pose = apply_transform_relative(request.pose, request.transform)
        response.success = True
        return response


def apply_transform(pose: PoseStamped, transform: Transform) -> PoseStamped:
    """Apply a transformation to a PoseStamped object.

    Args:
        pose (PoseStamped): The pose to be transformed.
        transform (Transform): The transformation to apply.

    Returns:
        PoseStamped: The transformed pose.
    """
    transform_stamped = TransformStamped()
    transform_stamped.transform = transform
    transform_stamped.header = pose.header
    return do_transform_pose_stamped(pose, transform_stamped)


def apply_transform_relative(pose: PoseStamped, transform: Transform) -> PoseStamped:
    """Apply a transformation to a PoseStamped object relative to its current position.

    Args:
        pose (PoseStamped): The pose to be transformed.
        transform (Transform): The transformation to apply.

    Returns:
        PoseStamped: The transformed pose with the original position added back.
    """
    start_position = copy(pose.pose.position)
    pose.pose.position = Point()
    pose = apply_transform(pose, transform)
    pose.pose.position.x += start_position.x
    pose.pose.position.y += start_position.y
    pose.pose.position.z += start_position.z
    return pose


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and start the executor.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    node = ManipulatePose()
    spin_node(node)


if __name__ == "__main__":
    main()
