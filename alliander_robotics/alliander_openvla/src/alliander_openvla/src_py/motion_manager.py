#!/usr/bin/env python3
# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0
# This class handles the motion of the Franka arm. 
# It takes action array as input and publishes twist/pose messages to the correct ROS topics. 

# Imports
from geometry_msgs.msg import TwistStamped, PoseStamped
from scipy.spatial.transform import Rotation
import rclpy
from alliander_openvla.utils import convert_posedelta_to_velo

class MotionManager:

    def __init__(self, node, pose_pub, twist_pub, tf_buffer):
        self.node = node
        self.pose_pub = pose_pub
        self.twist_pub = twist_pub
        self.tf_buffer = tf_buffer

    
    def execute_action(self, action, command_mode) -> None:
        if command_mode == "twist":
            twist_msg = self.create_twist_msg(action)
            self.twist_pub.publish(twist_msg)
            return None

        elif command_mode == "pose":
            pose_msg = self.create_pose_msg(action)
            self.pose_pub.publish(pose_msg)
            return pose_msg # pose msg is returned because it can be used to set the current target pose for visualization
 

    def create_twist_msg(self, action):
        linear, angular = convert_posedelta_to_velo(action)

        twist = TwistStamped()
        twist.header.stamp = self.node.get_clock().now().to_msg()
        twist.header.frame_id = "franka/fr3_hand_tcp"

        twist.twist.linear.x = linear[0]
        twist.twist.linear.y = linear[1]
        twist.twist.linear.z = linear[2]

        twist.twist.angular.x = angular[0]
        twist.twist.angular.y = angular[1]
        twist.twist.angular.z = angular[2]

        return twist
    
    
    def create_pose_msg(self, action):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = "franka/world"

        dx, dy, dz = action[:3]
        droll, dpitch, dyaw = action[3:6]

        try:
            transform = self.tf_buffer.lookup_transform(
                "franka/world",
                "franka/fr3_hand_tcp",
                rclpy.time.Time()
            )

            pos = transform.transform.translation
            rot = transform.transform.rotation

            current_rot = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w])
            delta_rot = Rotation.from_euler("xyz", [droll, dpitch, dyaw])

            target_rot = current_rot * delta_rot
            quat = target_rot.as_quat()

            delta_world = current_rot.apply([dx, dy, dz])

            pose_msg.pose.position.x = pos.x + delta_world[0]
            pose_msg.pose.position.y = pos.y + delta_world[1]
            pose_msg.pose.position.z = pos.z + delta_world[2]

            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]

        except Exception as e:
            self.node.get_logger().warn(f"Pose transform failed: {e}")

        return pose_msg


