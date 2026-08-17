#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import math
import time
from collections.abc import Callable
from copy import deepcopy
from typing import Any

import rclpy
from alliander_utilities.ros_utils import spin_executor
from geographic_msgs.msg import GeoPath, GeoPose, GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry, Path
from pyproj import CRS, Transformer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger


class Nav2Manager(Node):
    """ROS 2 node to manage navigation using Nav2 BasicNavigator."""

    def __init__(self) -> None:
        """Initialize the Nav2Manager node."""
        super().__init__("nav2_manager")
        self.basic_navigator: BasicNavigator = BasicNavigator()
        self.current_gps_fix: NavSatFix | None = None
        self.current_odometry: Odometry | None = None

        self.declare_parameter("costmap_size", 20.0)
        self.costmap_size = (
            self.get_parameter("costmap_size").get_parameter_value().double_value
        )

        self.create_subscription(PoseStamped, "/goal_pose", self.cb_goal_pose, 10)
        self.create_subscription(Path, "/waypoints", self.cb_waypoints, 10)
        self.create_subscription(GeoPath, "/gps_waypoints", self.cb_gps_waypoints, 10)
        self.create_subscription(NavSatFix, "gps/fix", self.cb_gps_fix, 10)
        self.create_subscription(Odometry, "odometry/filtered", self.cb_odometry, 10)
        self.create_service(Trigger, "~/stop", self.cb_stop)

        self.get_logger().info("Controller is ready.")

    def cb_goal_pose(self, msg: PoseStamped) -> None:
        """Callback on receiving a PoseStamped message with a goal pose.

        If the goal is farther away than the costmap allows for in one step, it is
        approached through a sequence of intermediate waypoints instead.

        Args:
            msg (PoseStamped): The received PoseStamped message.
        """
        self.get_logger().info("Received new goal pose for navigation.")
        poses = self._constrain_xy_waypoints([msg])
        if len(poses) > 1:  # noqa: PLR2004
            self.basic_navigator.followWaypoints(poses)
        else:
            self.basic_navigator.goToPose(msg)

    def cb_waypoints(self, msg: Path) -> None:
        """Callback on receiving a Path message with waypoints.

        Args:
            msg (Path): The received Path message.
        """
        self.get_logger().info("Received new waypoints for navigation.")
        poses = self._constrain_xy_waypoints(msg.poses)
        self.basic_navigator.followWaypoints(poses)

    def cb_odometry(self, msg: Odometry) -> None:
        """Callback on receiving the robot's current odometry.

        Args:
            msg (Odometry): The received Odometry message.
        """
        self.current_odometry = msg

    def cb_gps_waypoints(self, msg: GeoPath) -> None:
        """Callback on receiving a GeoPoseStamped message with GPS waypoints.

        Args:
            msg (GeoPath): The received GeoPath message.
        """
        self.get_logger().info("Received new GPS waypoints for navigation.")
        geo_poses = []
        for geo_pose_stamped in msg.poses:
            geo_pose_stamped: GeoPoseStamped
            geo_poses.append(geo_pose_stamped.pose)
        geo_poses = self._constrain_geo_waypoints(geo_poses)
        self.basic_navigator.followGpsWaypoints(geo_poses)

    def cb_gps_fix(self, msg: NavSatFix) -> None:
        """Callback on receiving the robot's current GPS fix.

        Args:
            msg (NavSatFix): The received NavSatFix message.
        """
        self.current_gps_fix = msg

    def cb_stop(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Stop any active navigation goal.

        Args:
            response (Trigger.Response): The response object.

        Returns:
            Trigger.Response: The response object.
        """
        active_task = not self.basic_navigator.isTaskComplete()
        if not active_task:
            response.message = "No active navigation task to stop."
            response.success = True
            self.get_logger().info(response.message)
            return response

        start = time.time()
        timeout = 3.0  # seconds
        self.basic_navigator.cancelTask()
        while time.time() - start < timeout:
            if self.basic_navigator.isTaskComplete():
                response.message = "Successfully cancelled the active navigation task."
                response.success = True
                self.get_logger().info(response.message)
                return response
            time.sleep(0.1)

        response.message = "Failed to cancel the active navigation task within timeout."
        response.success = False
        self.get_logger().warn(response.message)
        return response

    def _constrain_positions(
        self,
        poses: list[Any],
        current_position: tuple[float, float] | None,
        get_xy: Callable[[Any], tuple[float, float]],
        make_intermediate: Callable[[Any, float, float], Any],
        no_position_warning: str,
    ) -> list[Any]:
        """Ensure consecutive positions stay within costmap bounds of each other.

        Args:
            poses (list[Any]): original list of poses.
            current_position (tuple[float, float] | None): the robot's current
                (x, y) position, in the same local frame as get_xy and
                make_intermediate operate in, or None if not yet known.
            get_xy (Callable[[Any], tuple[float, float]]): extracts the local
                (x, y) position from a pose.
            make_intermediate (Callable[[Any, float, float], Any]): builds an
                intermediate pose at the given local (x, y) position, based on a
                deep copy of the target pose that follows it.
            no_position_warning (str): logged when current_position is None and
                fewer than two poses were received, since interpolation is then
                impossible.

        Returns:
            list[Any]: the modified list of poses.
        """
        if not poses:
            return poses

        if current_position is not None:
            origin = current_position
            targets = poses
            constrained_poses = []
        elif len(poses) >= 2:  # noqa: PLR2004
            origin = get_xy(poses[0])
            targets = poses[1:]
            constrained_poses = [poses[0]]
        else:
            self.get_logger().warn(no_position_warning)
            return poses

        max_step = (self.costmap_size / 2.0) * 0.95
        start_x, start_y = origin

        for end_pose in targets:
            end_x, end_y = get_xy(end_pose)

            dx = end_x - start_x
            dy = end_y - start_y
            distance = math.hypot(dx, dy)

            if distance > max_step:
                num_segments = math.ceil(distance / max_step)

                for segment in range(1, num_segments):
                    ratio = segment / num_segments

                    intermediate_x = start_x + dx * ratio
                    intermediate_y = start_y + dy * ratio

                    constrained_poses.append(
                        make_intermediate(end_pose, intermediate_x, intermediate_y)
                    )

            constrained_poses.append(end_pose)
            start_x, start_y = end_x, end_y

        self.get_logger().info(
            f"Original positions: {[get_xy(p) for p in poses]}, "
            f"adjusted positions: {[get_xy(p) for p in constrained_poses]}"
        )
        return constrained_poses

    def _constrain_xy_waypoints(self, poses: list[PoseStamped]) -> list[PoseStamped]:
        """Ensure waypoints stay within costmap bounds of each other.

        Args:
            poses (list[PoseStamped]): original list of poses.

        Returns:
            list[PoseStamped]: the modified list of poses.
        """
        current_odometry = self.current_odometry
        current_position = (
            (
                current_odometry.pose.pose.position.x,
                current_odometry.pose.pose.position.y,
            )
            if current_odometry is not None
            else None
        )

        def get_xy(pose: PoseStamped) -> tuple[float, float]:
            return pose.pose.position.x, pose.pose.position.y

        def make_intermediate(end_pose: PoseStamped, x: float, y: float) -> PoseStamped:
            intermediate_pose = deepcopy(end_pose)
            intermediate_pose.pose.position.x = x
            intermediate_pose.pose.position.y = y
            return intermediate_pose

        return self._constrain_positions(
            poses,
            current_position,
            get_xy,
            make_intermediate,
            "No odometry received yet, cannot check distance to first waypoint.",
        )

    def _constrain_geo_waypoints(self, geo_poses: list[GeoPose]) -> list[GeoPose]:
        """Ensure GPS waypoints stay within costmap bounds of each other.

        Args:
            geo_poses (list[GeoPose]): original list of GeoPoses.

        Returns:
            list[GeoPose]: the modified list of GeoPoses.
        """
        if not geo_poses:
            return geo_poses

        current_fix = self.current_gps_fix
        if current_fix is not None:
            origin_lat, origin_lon = current_fix.latitude, current_fix.longitude
        elif len(geo_poses) >= 2:  # noqa: PLR2004
            origin_lat = geo_poses[0].position.latitude
            origin_lon = geo_poses[0].position.longitude
        else:
            self.get_logger().warn(
                "No GPS fix received yet, cannot check distance to first waypoint."
            )
            return geo_poses

        local_crs = CRS.from_proj4(
            f"+proj=aeqd +lat_0={origin_lat} "
            f"+lon_0={origin_lon} +datum=WGS84 +units=m +no_defs"
        )
        wgs84 = CRS.from_epsg(4326)

        to_local = Transformer.from_crs(wgs84, local_crs, always_xy=True)
        to_wgs84 = Transformer.from_crs(local_crs, wgs84, always_xy=True)

        current_position = (
            to_local.transform(current_fix.longitude, current_fix.latitude)
            if current_fix is not None
            else None
        )

        def get_xy(pose: GeoPose) -> tuple[float, float]:
            return to_local.transform(pose.position.longitude, pose.position.latitude)

        def make_intermediate(end_pose: GeoPose, x: float, y: float) -> GeoPose:
            lon, lat = to_wgs84.transform(x, y)
            intermediate_pose = deepcopy(end_pose)
            intermediate_pose.position.latitude = lat
            intermediate_pose.position.longitude = lon
            return intermediate_pose

        return self._constrain_positions(
            geo_poses,
            current_position,
            get_xy,
            make_intermediate,
            "No GPS fix received yet, cannot check distance to first waypoint.",
        )


def main(args: list | None = None) -> None:
    """Main function to initialize the ROS 2 node and set the thresholds.

    Args:
        args (list | None): Command line arguments, defaults to None.
    """
    rclpy.init(args=args)
    node = Nav2Manager()

    # Use an executor to avoid problems with BasicNavigator's internal spinning:
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_executor(executor)


if __name__ == "__main__":
    main()
