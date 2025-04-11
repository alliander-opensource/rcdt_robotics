// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>

typedef sensor_msgs::msg::Image Image;

class Convert32FC1to16UC1 : public rclcpp::Node {
public:
  Convert32FC1to16UC1();

private:
  rclcpp::Subscription<Image>::SharedPtr subscription;
  rclcpp::Publisher<Image>::SharedPtr publisher;
  void subscription_callback(Image msg);
};