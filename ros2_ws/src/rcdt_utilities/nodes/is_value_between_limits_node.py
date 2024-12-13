#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy import logging
from rclpy.node import Node

from rcdt_utilities_msgs.srv import IsValueBetweenLimits

ros_logger = logging.get_logger(__name__)


class IsValueBetweenLimitsNode(Node):
    def __init__(self) -> None:
        super().__init__("is_value_between_limits")
        self.create_service(
            IsValueBetweenLimits, "/is_value_between_limits", self.callback
        )

    def callback(
        self,
        request: IsValueBetweenLimits.Request,
        response: IsValueBetweenLimits.Response,
    ) -> IsValueBetweenLimits.Response:
        """Determines if given value is between lower and upper limit.

        Always returns success=True.

        """
        is_between = request.lower_limit < request.input < request.upper_limit
        ros_logger.info(f"Is between: {is_between}")

        response.is_between = is_between
        response.success = True
        return response


def main(args: str = None) -> None:
    rclpy.init(args=args)
    try:
        node = IsValueBetweenLimitsNode()
        rclpy.spin(node)
    except Exception as e:
        ros_logger.error(e)
        raise e
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
