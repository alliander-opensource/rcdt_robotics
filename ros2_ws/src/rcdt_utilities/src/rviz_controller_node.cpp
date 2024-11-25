// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "rclcpp/rclcpp.hpp"
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
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_markers", this));
    srv_add_marker_ = create_service<rcdt_utilities_msgs::srv::AddMarker>(
        "~/add_marker", std::bind(&MinimalPublisher::add_marker, this, std::placeholders::_1, std::placeholders::_2));
    srv_clear_all_ = create_service<std_srvs::srv::Trigger>(
        "~/clear_all", std::bind(&MinimalPublisher::clear_all, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
  rclcpp::Service<rcdt_utilities_msgs::srv::AddMarker>::SharedPtr srv_add_marker_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_all_;

  void add_marker(const std::shared_ptr<rcdt_utilities_msgs::srv::AddMarker::Request> request,
                  std::shared_ptr<rcdt_utilities_msgs::srv::AddMarker::Response> response)
  {
    visual_tools_->publishAxis(request->marker_pose.pose);
    visual_tools_->trigger();
  }

  void clear_all(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
}