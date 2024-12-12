#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy import logging
from rclpy.node import Node
from rclpy import wait_for_message
from rcdt_detection_msgs.srv import GetRGBDFromTopic
from realsense2_camera_msgs.msg import RGBD

ros_logger = logging.get_logger(__name__)


class GetRGBDFromTopicNode(Node):
    def __init__(self) -> None:
        super().__init__("get_rgbd_from_topic")
        self.create_service(GetRGBDFromTopic, "/get_rgbd_from_topic", self.callback)

    def callback(
        self, request: GetRGBDFromTopic.Request, response: GetRGBDFromTopic.Response
    ) -> GetRGBDFromTopic.Response:
        response.success, message = wait_for_message.wait_for_message(
            RGBD, self, request.topic, time_to_wait=1
        )
        if response.success:
            response.rgbd = message
        else:
            ros_logger.error(f"No RGBD message received on {request.topic}.")
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    node = GetRGBDFromTopicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        ros_logger.info("Keyboard interrupt, shutting down.\n")
    except Exception as e:
        raise e
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
