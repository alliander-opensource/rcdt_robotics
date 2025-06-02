// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "combine_camera_topics.hpp"
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/parameter_value.hpp>

using std::placeholders::_1;

CombineCameraTopics::CombineCameraTopics() : Node("combine_camera_topics") {

  this->declare_parameter("rgb_topic", "/camera/camera/color/image_raw");
  this->declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw");
  this->declare_parameter("rgb_info_topic", "/camera/camera/color/camera_info");
  this->declare_parameter("depth_info_topic",
                          "/camera/camera/depth/camera_info");
  this->declare_parameter("rgbd_topic", "/camera/camera/rgbd");
  this->declare_parameter("rate", 30);

  initialize_subscriptions();
  initialize_publisher();
};

void CombineCameraTopics::initialize_subscriptions() {
  auto rgb_topic = this->get_parameter("rgb_topic").as_string();
  sub_rgb = this->create_subscription<Image>(
      rgb_topic, 10, std::bind(&CombineCameraTopics::rgb_callback, this, _1));
  auto depth_topic = this->get_parameter("depth_topic").as_string();
  sub_depth = this->create_subscription<Image>(
      depth_topic, 10,
      std::bind(&CombineCameraTopics::depth_callback, this, _1));
  auto rgb_info_topic = this->get_parameter("rgb_info_topic").as_string();
  sub_rgb_info = this->create_subscription<CameraInfo>(
      rgb_info_topic, 10,
      std::bind(&CombineCameraTopics::rgb_info_callback, this, _1));
  auto depth_info_topic = this->get_parameter("depth_info_topic").as_string();
  sub_depth_info = this->create_subscription<CameraInfo>(
      depth_info_topic, 10,
      std::bind(&CombineCameraTopics::depth_info_callback, this, _1));
}

void CombineCameraTopics::initialize_publisher() {
  auto rgbd_topic = this->get_parameter("rgbd_topic").as_string();
  pub_rgbd = this->create_publisher<RGBD>(rgbd_topic, 10);
  auto rate = this->get_parameter("rate").as_int();
  timer = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&CombineCameraTopics::timer_callback, this));
}

void CombineCameraTopics::rgb_callback(Image msg) { rgbd.rgb = msg; }
void CombineCameraTopics::depth_callback(Image msg) { rgbd.depth = msg; }
void CombineCameraTopics::rgb_info_callback(CameraInfo msg) {
  rgbd.rgb_camera_info = msg;
}
void CombineCameraTopics::depth_info_callback(CameraInfo msg) {
  rgbd.depth_camera_info = msg;
}

void CombineCameraTopics::timer_callback() { pub_rgbd->publish(rgbd); };

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CombineCameraTopics>());
  rclcpp::shutdown();
}