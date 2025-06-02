// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <franka_msgs/action/move.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

typedef std_srvs::srv::Trigger Trigger;
typedef franka_msgs::action::Move Move;

class OpenGripper : public rclcpp::Node {
public:
  OpenGripper();

private:
  rclcpp::Service<Trigger>::SharedPtr service;
  rclcpp_action::Client<Move>::SharedPtr client;
  void callback(const std::shared_ptr<Trigger::Request> request,
                std::shared_ptr<Trigger::Response> response);
};