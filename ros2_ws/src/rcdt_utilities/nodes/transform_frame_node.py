#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy import time
from rclpy import logging
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from rcdt_utilities_msgs.srv import TransformPose

ros_logger = logging.get_logger(__name__)


class TransformPoseNode(Node):
    def __init__(self):
        super().__init__("transform_frame")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_service(TransformPose, "/transform_frame", self.callback)

    def callback(
        self, request: TransformPose.Request, response: TransformPose.Response
    ) -> TransformPose.Response:
        target_frame = request.target_frame
        source_frame = request.pose_in.header.frame_id

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {target_frame} to {source_frame}: {ex}"
            )
            response.success = False
            return response

        response.pose_out = do_transform_pose_stamped(request.pose_in, transform)
        response.success = True
        return response


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
