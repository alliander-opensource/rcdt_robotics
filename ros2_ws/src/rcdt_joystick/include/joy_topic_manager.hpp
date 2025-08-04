// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <nlohmann/json.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joy.hpp>

using json = nlohmann::json;
using Joy = sensor_msgs::msg::Joy;

struct Button {
  int key;
  std::string topic;
  std::string service;
  int state = -1;
};

class JoyTopicManager : public rclcpp::Node {
public:
  std::string package = "rcdt_joystick";
  std::string file_name = "/config/gamepad_mapping.json";
  JoyTopicManager();

private:
  std::map<int, Button> buttons;

  rclcpp::Subscription<Joy>::SharedPtr subscription_joy;
  void callback_joy(Joy msg);

  void read_json();
  void add_button(json json);
};