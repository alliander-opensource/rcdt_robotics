#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from copy import copy
from rclpy import time
from rclpy import logging
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped
from rcdt_utilities_msgs.srv import TransformPose

ros_logger = logging.get_logger(__name__)


class TransformPoseNode(Node):
    def __init__(self):
        super().__init__("transform_pose")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_service(TransformPose, "/transform_pose", self.callback)

    def callback(
        self, request: TransformPose.Request, response: TransformPose.Response
    ) -> TransformPose.Response:
        pose = request.pose_in
        if request.target_frame != "":
            pose = self.change_frame(pose, request.target_frame)
            if pose is None:
                response.success = False
                return response
        response.pose_out = self.apply_transform(pose, request.transform)
        response.success = True
        return response

    def change_frame(self, pose: PoseStamped, target_frame: str) -> PoseStamped:
        source_frame = pose.header.frame_id

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {target_frame} to {source_frame}: {ex}"
            )
            return None
        return do_transform_pose_stamped(pose, transform)

    def apply_transform(self, pose: PoseStamped, transform: Transform) -> PoseStamped:
        start_position = copy(pose.pose.position)
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        transform_stamped = TransformStamped()
        transform_stamped.transform = transform
        transform_stamped.header = pose.header
        pose = do_transform_pose_stamped(pose, transform_stamped)
        pose.pose.position.x += start_position.x
        pose.pose.position.y += start_position.y
        pose.pose.position.z += start_position.z
        return pose


def main(args: str = None) -> None:
    rclpy.init(args=args)
    try:
        node = TransformPoseNode()
        rclpy.spin(node)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
