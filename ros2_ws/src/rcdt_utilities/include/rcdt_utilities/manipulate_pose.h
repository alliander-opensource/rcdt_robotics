// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcdt_messages/srv/express_pose_in_other_frame.hpp"
#include "rcdt_messages/srv/transform_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.hpp>

class PoseManipulator : public rclcpp::Node {
public:
  PoseManipulator();

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Services
  rclcpp::Service<rcdt_messages::srv::ExpressPoseInOtherFrame>::SharedPtr
      express_pose_in_other_frame_service_;
  void expressPoseInOtherFrame(
      const std::shared_ptr<
          rcdt_messages::srv::ExpressPoseInOtherFrame::Request>
          req,
      std::shared_ptr<rcdt_messages::srv::ExpressPoseInOtherFrame::Response>
          resp);

  rclcpp::Service<rcdt_messages::srv::TransformPose>::SharedPtr
      transform_pose_service_;
  void transformPose(
      const std::shared_ptr<rcdt_messages::srv::TransformPose::Request> req,
      std::shared_ptr<rcdt_messages::srv::TransformPose::Response> resp);

  rclcpp::Service<rcdt_messages::srv::TransformPose>::SharedPtr
      transform_pose_relative_service_;
  void transformPoseRelative(
      const std::shared_ptr<rcdt_messages::srv::TransformPose::Request> req,
      std::shared_ptr<rcdt_messages::srv::TransformPose::Response> resp);

  geometry_msgs::msg::PoseStamped
  doTransform(const geometry_msgs::msg::PoseStamped pose_in,
              const geometry_msgs::msg::Transform tf);
};
