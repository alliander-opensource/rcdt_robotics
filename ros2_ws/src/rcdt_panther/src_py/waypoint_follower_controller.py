#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from numpy import deg2rad
from rcdt_utilities.launch_utils import get_file_path, get_yaml, spin_executor
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler


class WaypointFollowerController(Node):
    """Node to follow waypoints for the robot."""

    def __init__(self, executor: MultiThreadedExecutor) -> None:
        """Initialize the WaypointFollowerController node.

        Args:
            executor (MultiThreadedExecutor): The executor is required for the spin_until_future_complete calls, otherwise a second call never finishes.
        """
        super().__init__("waypoint_follower_controller")

        self.executor = executor

        cb_start = MutuallyExclusiveCallbackGroup()
        cb_stop = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "~/start", self.start, callback_group=cb_start)
        self.create_service(Trigger, "~/stop", self.stop, callback_group=cb_stop)

        self.client = ActionClient(self, FollowWaypoints, "/follow_waypoints")

        self.goal = FollowWaypoints.Goal()
        self.goal_handle: ClientGoalHandle | None = None

        self.get_logger().info("Controller is ready.")

    def start(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Callback on the start service call.

        Args:
            response (Trigger.Response): The response object.

        Returns:
            Trigger.Response: The response object.
        """
        response.success = self.start_action_server()
        return response

    def stop(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Callback on the stop service call.

        Args:
            response (Trigger.Response): The response object.

        Returns:
            Trigger.Response: The response object.
        """
        response.success = self.stop_action_server()
        return response

    def start_action_server(self) -> bool:
        """Update the goal and start the action server.

        Returns:
            bool: True if the action server was started successfully, False otherwise.
        """
        self.update_goal()
        if not self.client.wait_for_server(1):
            self.get_logger().error("Failed to connect to action server.")
            return False
        future_goal = self.client.send_goal_async(self.goal)
        rclpy.spin_until_future_complete(self, future_goal, self.executor, 1)
        if not future_goal.done():
            self.get_logger().error("Failed to send goal.")
            return False
        goal_handle: ClientGoalHandle = future_goal.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was not accepted.")
            return False
        self.get_logger().info("Navigation to waypoints is started.")
        self.goal_handle = goal_handle
        return True

    def stop_action_server(self) -> bool:
        """Stop the action server.

        Returns:
            bool: True if the action server was stopped successfully, False otherwise.
        """
        if self.goal_handle is None:
            self.get_logger().warning("No active goal to stop.")
            return False
        self.future = self.client._cancel_goal_async(self.goal_handle)
        rclpy.spin_until_future_complete(self, self.future, self.executor, 1)
        if not self.future.done():
            self.get_logger().error("Failed to stop goal.")
            return False
        self.get_logger().info("Navigation to waypoints is stopped.")
        self.goal_handle = None
        return True

    def update_goal(self) -> None:
        """Update the goal for the action server."""
        self.goal = FollowWaypoints.Goal()
        file_path = get_file_path("rcdt_panther", ["config"], "waypoints.yaml")
        yaml = get_yaml(file_path)
        self.goal.number_of_loops = yaml["loops"]
        frame = yaml["frame"]
        waypoints = yaml["waypoints"]
        self.goal.poses = [waypoint_to_pose(frame, waypoint) for waypoint in waypoints]


def waypoint_to_pose(frame: str, waypoint: list[float]) -> PoseStamped:
    """Convert a waypoint to a PoseStamped message.

    Args:
        frame (str): The frame ID for the pose.
        waypoint (list[float]): The waypoint coordinates [x (m), y (m), rotation (degrees)].

    Returns:
        PoseStamped: The converted PoseStamped message.
    """
    x, y, rotation = waypoint

    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y

    quaternion = quaternion_from_euler(0.0, 0.0, deg2rad(rotation))
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    return pose


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and set the thresholds.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = WaypointFollowerController(executor)
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
