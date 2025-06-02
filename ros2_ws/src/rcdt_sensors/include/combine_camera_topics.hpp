// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

typedef sensor_msgs::msg::Image Image;
typedef sensor_msgs::msg::CameraInfo CameraInfo;
typedef realsense2_camera_msgs::msg::RGBD RGBD;

class CombineCameraTopics : public rclcpp::Node {
public:
  CombineCameraTopics();

private:
  void declare_parameters();
  void initialize_subscriptions();
  void initialize_publisher();

  std::string rgb_topic;
  std::string depth_topic;
  std::string rgb_info_topic;
  std::string depth_info_topic;

  rclcpp::Subscription<Image>::SharedPtr sub_rgb;
  rclcpp::Subscription<Image>::SharedPtr sub_depth;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_rgb_info;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_depth_info;

  void rgb_callback(const Image msg);
  void depth_callback(const Image msg);
  void rgb_info_callback(const CameraInfo msg);
  void depth_info_callback(const CameraInfo msg);

  rclcpp::Publisher<RGBD>::SharedPtr pub_rgbd;
  RGBD rgbd;

  rclcpp::TimerBase::SharedPtr timer;
  void timer_callback();
};