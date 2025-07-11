// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <nlohmann/json.hpp>
#include <rclcpp/node.hpp>

using json = nlohmann::json;

struct Button {
  int key;
  std::string topic;
  std::string service;
};

class JoyTopicManager : public rclcpp::Node {
public:
  std::string package = "rcdt_joystick";
  std::string file_name = "/config/gamepad_mapping.json";
  JoyTopicManager();

private:
  std::map<int, Button> buttons;
  void read_json();
  void add_button(json json);
};