// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "open_gripper.hpp"
#include <functional>
#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

OpenGripper::OpenGripper() : Node("open_gripper") {
  service = this->create_service<Trigger>(
      "open_gripper", std::bind(&OpenGripper::callback, this, _1, _2));
  client = rclcpp_action::create_client<Move>(this, "/franka/fr3_gripper/move");
};

void OpenGripper::callback(const std::shared_ptr<Trigger::Request> request,
                           std::shared_ptr<Trigger::Response> response) {
  if (!client->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(this->get_logger(), "Gripper move client not available.");
    return;
  }

  auto goal = Move::Goal();
  goal.width = 0.08;
  goal.speed = 0.03;

  auto options = rclcpp_action::Client<Move>::SendGoalOptions();
  options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<Move>::SharedPtr handle) {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server.");
        }
      };
  options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<Move>::SharedPtr handle,
             const std::shared_ptr<const Move::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %f",
                    feedback->current_width);
      };
  options.result_callback =
      [this](
          const rclcpp_action::ClientGoalHandle<Move>::WrappedResult &result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Gripper opened successfully.");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Gripper opening aborted.");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Gripper opening canceled.");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
          break;
        }
      };

  client->async_send_goal(goal, options);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenGripper>());
  rclcpp::shutdown();
}