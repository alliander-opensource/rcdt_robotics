// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rviz_visual_tools/rviz_visual_tools.hpp"
#include "rcdt_utilities_msgs/srv/add_marker.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <memory>

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("rviz_controller")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_markers", this));
    srv_add_marker_ = create_service<rcdt_utilities_msgs::srv::AddMarker>(
        "/add_marker", std::bind(&MinimalPublisher::add_marker, this, std::placeholders::_1, std::placeholders::_2));
    srv_clear_all_ = create_service<std_srvs::srv::Trigger>(
        "/clear_all", std::bind(&MinimalPublisher::clear_all, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  rclcpp::Service<rcdt_utilities_msgs::srv::AddMarker>::SharedPtr srv_add_marker_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_all_;

  void add_marker(const std::shared_ptr<rcdt_utilities_msgs::srv::AddMarker::Request> request,
                  std::shared_ptr<rcdt_utilities_msgs::srv::AddMarker::Response> response)
  {
    if (request->marker_pose.header.frame_id != "world")
    {
      try
      {
        request->marker_pose = change_to_world_frame(request->marker_pose);
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_ERROR(this->get_logger(), "Could not add marker since transformation failed.");
        return;
      }
    };
    visual_tools_->publishAxis(request->marker_pose.pose);
    visual_tools_->trigger();
    response->success = true;
  }

  void clear_all(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
  }

  geometry_msgs::msg::PoseStamped change_to_world_frame(geometry_msgs::msg::PoseStamped pose)
  {
    geometry_msgs::msg::TransformStamped transform;
    std::string source_frame = pose.header.frame_id.c_str();
    std::string target_frame = "world";
    try
    {
      transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(
          this->get_logger(), "Could not transform %s to %s: %s",
          source_frame.c_str(), target_frame.c_str(), ex.what());
      throw;
      return pose;
    }
    tf2::doTransform(pose, pose, transform);
    return pose;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
}