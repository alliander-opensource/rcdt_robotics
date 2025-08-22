#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rcdt_utilities.launch_utils import spin_node
from rclpy.action import ActionClient
from rclpy.node import Node


class WaypointFollower(Node):
    """Node to follow waypoints for the robot."""

    def __init__(self):
        """Initialize the WaypointFollower node."""
        super().__init__("waypoint_follower")
        self.action_client = ActionClient(self, FollowWaypoints, "/follow_waypoints")

        goal = FollowWaypoints.Goal()

        corner = 3.0

        pose1 = PoseStamped()
        pose1.header.frame_id = "map"
        pose1.pose.position.x = corner
        pose1.pose.position.y = corner

        pose2 = PoseStamped()
        pose2.header.frame_id = "map"
        pose2.pose.position.x = corner
        pose2.pose.position.y = -corner
        pose2.pose.orientation.w = -0.707
        pose2.pose.orientation.z = 0.707

        pose3 = PoseStamped()
        pose3.header.frame_id = "map"
        pose3.pose.position.x = -corner
        pose3.pose.position.y = -corner
        pose3.pose.orientation.w = 0.0
        pose3.pose.orientation.z = 1.0

        pose4 = PoseStamped()
        pose4.header.frame_id = "map"
        pose4.pose.position.x = -corner
        pose4.pose.position.y = corner
        pose4.pose.orientation.w = 0.707
        pose4.pose.orientation.z = 0.707

        goal.poses = [pose1, pose2, pose3, pose4]
        goal.number_of_loops = 10
        self.action_client.send_goal_async(goal)


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and set the thresholds.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    node = WaypointFollower()
    node.get_logger().info("Waypoint Follower Node has been started.")
    spin_node(node)


if __name__ == "__main__":
    main()
