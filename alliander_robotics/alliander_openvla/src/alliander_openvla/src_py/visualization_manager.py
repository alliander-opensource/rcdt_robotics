#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
# This class handles the publishing of visualization markers and annotated images to RViz.

# Imports
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import rclpy
import cv2
import yaml
from alliander_openvla.utils import cv_to_ros_image
from std_msgs.msg import String
import numpy as np


class VisualizationManager:

    def __init__(self, node, marker_pub, image_pub):
        self.node = node
        self.marker_pub = marker_pub
        self.image_pub = image_pub
        
        self.frame_id = "/franka/world"

        self.marker_id = 0

        self.traj_points = []
        self.max_traj_length = 50

    # def clear_markers(self):
    #     marker


    def publish_markers(self, ee_pose, target_pose, action, task) -> None:
        """Publish 3D markers for RViz based on EE delta actions."""

        marker_array = MarkerArray()

        # Lookup current EE pose is replaced by the ee_pos we pass
        ee_x, ee_y, ee_z = ee_pose

        # Same for target pose
        target_x, target_y, target_z = target_pose

        # Header
        stamp = self.node.get_clock().now().to_msg()

        # Arrow: EE --> Target
        arrow = Marker()
        arrow.header.frame_id = "/franka/world"
        arrow.header.stamp = stamp
        arrow.ns = "vla_waypoints"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        # REQUIRED when using points
        arrow.pose.orientation.w = 1.0

        # Define arrow start and end
        arrow.points = [
            Point(x=ee_x, y=ee_y, z=ee_z),
            Point(x=target_x, y=target_y, z=target_z)
        ]

        # Arrow thickness
        arrow.scale.x = 0.02  # shaft diameter
        arrow.scale.y = 0.04  # head diameter
        arrow.scale.z = 0.08  # head length

        arrow.color.r = 0.0
        arrow.color.g = 1.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0

        arrow.lifetime.sec = 0

        # Clear markers before appending any of the actual markers
        clear_marker = Marker()
        clear_marker.header.frame_id = self.frame_id
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL

        marker_array.markers.append(clear_marker)

        marker_array.markers.append(arrow)

        # Optional: EE sphere (nice debug)
        ee_marker = Marker()
        ee_marker.header = arrow.header
        ee_marker.ns = "ee_position"
        ee_marker.id = 2
        ee_marker.type = Marker.SPHERE
        ee_marker.action = Marker.ADD

        ee_marker.pose.position.x = ee_x
        ee_marker.pose.position.y = ee_y
        ee_marker.pose.position.z = ee_z
        ee_marker.pose.orientation.w = 1.0

        ee_marker.scale.x = 0.04
        ee_marker.scale.y = 0.04
        ee_marker.scale.z = 0.04

        ee_marker.color.r = 0.0
        ee_marker.color.g = 0.0
        ee_marker.color.b = 1.0
        ee_marker.color.a = 1.0

        marker_array.markers.append(ee_marker)

        # Optional: Target sphere (nice debug)
        target_marker = Marker()
        target_marker.header = arrow.header
        target_marker.ns = "target_position"
        target_marker.id = 3
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD

        target_marker.pose.position.x = target_x
        target_marker.pose.position.y = target_y
        target_marker.pose.position.z = target_z
        target_marker.pose.orientation.w = 1.0

        target_marker.scale.x = 0.04
        target_marker.scale.y = 0.04
        target_marker.scale.z = 0.04

        target_marker.color.r = 1.0
        target_marker.color.g = 0.0
        target_marker.color.b = 0.0
        target_marker.color.a = 1.0

        marker_array.markers.append(target_marker)

        # Text label at target
        text_marker = Marker()
        text_marker.header = arrow.header
        text_marker.ns = "vla_labels"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = target_x
        text_marker.pose.position.y = target_y
        text_marker.pose.position.z = target_z + 0.1
        text_marker.pose.orientation.w = 1.0

        text_marker.scale.z = 0.05
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = task

        marker_array.markers.append(text_marker)

        # Add trajectory plot
        self.traj_points.append(Point(x=target_x, y=target_y, z=target_z))

        if len(self.traj_points) > self.max_traj_length:
            self.traj_points.pop(0) # Keep buffer limited

        traj_marker = Marker()
        traj_marker.header = arrow.header
        traj_marker.ns = "trajectory"
        traj_marker.id = 4
        traj_marker.type = Marker.LINE_STRIP
        traj_marker.action = Marker.ADD

        traj_marker.points = self.traj_points

        traj_marker.scale.x = 0.01  # line width

        traj_marker.color.r = 1.0
        traj_marker.color.g = 1.0
        traj_marker.color.b = 0.0
        traj_marker.color.a = 1.0

        marker_array.markers.append(traj_marker)

        # Debug logging
        self.node.get_logger().info(
            f"EE: ({ee_x:.2f}, {ee_y:.2f}, {ee_z:.2f}) | "
            f"Target: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})"
        )

        # Publish
        self.marker_pub.publish(marker_array)
        

    def publish_annotated_image(self, cv_image: np.ndarray, action: np.ndarray, result: dict) -> None:
        """Publish annotated 2D image."""
        annotated = cv_image.copy()
        h, w = annotated.shape[:2]

        # Draw target point (project 3D to 2D - simplified)
        target_x = int((action[0] + 1) * w / 2)
        target_y = int((action[1] + 1) * h / 2)

        # Draw crosshair
        cv2.circle(annotated, (target_x, target_y), 10, (0, 255, 0), 2)
        cv2.line(annotated, (target_x - 15, target_y), (target_x + 15, target_y), (0, 255, 0), 2)
        cv2.line(annotated, (target_x, target_y - 15), (target_x, target_y + 15), (0, 255, 0), 2)

        # Add text overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(annotated, f"Task: {self.node.current_task}", (10, 30),
                    font, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated, f'Inference: {result["inference_time"] * 1000:.1f}ms',
                    (10, 60), font, 0.6, (255, 255, 255), 1)
        cv2.putText(annotated, f"Target: ({action[0]:.2f}, {action[1]:.2f}, {action[2]:.2f})",
                    (10, 90), font, 0.6, (255, 255, 255), 1)

        # Convert back to ROS message
        annotated_msg = cv_to_ros_image(annotated)
        annotated_msg.header.stamp = self.node.get_clock().now().to_msg()
        annotated_msg.header.frame_id = "camera"

        self.node.get_logger().info("Publishing annotated image...")
        self.image_pub.publish(annotated_msg)