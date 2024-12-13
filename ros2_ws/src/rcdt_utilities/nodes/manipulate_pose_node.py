#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from copy import copy
from rclpy import time, logging
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped, Point
from rcdt_utilities_msgs.srv import ExpressPoseInOtherFrame, TransformPose

ros_logger = logging.get_logger(__name__)


class ManipulatePoseNode(Node):
    def __init__(self):
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

    def transform_pose(
        self, request: TransformPose.Request, response: TransformPose.Response
    ) -> TransformPose.Response:
        response.pose = apply_transform(request.pose, request.transform)
        response.success = True
        return response

    def transform_pose_relative(
        self, request: TransformPose.Request, response: TransformPose.Response
    ) -> TransformPose.Response:
        response.pose = apply_transform_relative(request.pose, request.transform)
        response.success = True
        return response


def apply_transform(pose: PoseStamped, transform: Transform) -> PoseStamped:
    transform_stamped = TransformStamped()
    transform_stamped.transform = transform
    transform_stamped.header = pose.header
    return do_transform_pose_stamped(pose, transform_stamped)


def apply_transform_relative(pose: PoseStamped, transform: Transform) -> PoseStamped:
    start_position = copy(pose.pose.position)
    pose.pose.position = Point()
    pose = apply_transform(pose, transform)
    pose.pose.position.x += start_position.x
    pose.pose.position.y += start_position.y
    pose.pose.position.z += start_position.z
    return pose


def main(args: str = None) -> None:
    rclpy.init(args=args)
    try:
        node = ManipulatePoseNode()
        rclpy.spin(node)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
