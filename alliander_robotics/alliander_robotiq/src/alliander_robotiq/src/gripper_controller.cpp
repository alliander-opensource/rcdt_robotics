// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#include "alliander_robotiq/gripper_controller.hpp"

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stdexcept>
#include <string>
#include <vector>

RobotiqController::RobotiqController() : Node("gripper_controller") {
  this->declare_parameter("simulation", false);
  this->declare_parameter("ip", "");
  this->declare_parameter("port", -1);

  const bool simulation = this->get_parameter("simulation").as_bool();
  const std::string ip = this->get_parameter("ip").as_string();
  const int port = this->get_parameter("port").as_int();

  if (!simulation && (ip.empty() || port < 0)) {
    throw std::runtime_error(
        "IP address and port must be set in the parameters when using "
        "hardware.");
  }

  status_publisher_ =
      this->create_publisher<std_msgs::msg::String>("status", 10);
  joint_state_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  simulation_controller_publisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "position_controller/commands", 10);

  if (simulation) {
    controller_ = std::make_unique<ModbusControllerSimulation>(
        simulation_controller_publisher_);
  } else {
    controller_ = std::make_unique<ModbusControllerHardware>(ip, port);
    RCLCPP_INFO(this->get_logger(), "Connecting with gripper...");
    if (!controller_->open()) {
      RCLCPP_ERROR(this->get_logger(), "Connection failed.");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Connection successful.");
    }
  }

  timer_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = this->create_wall_timer(
      update_rate_, std::bind(&RobotiqController::update_callback, this),
      timer_callback_group_);

  controller_->send_command();

  action_server_ = rclcpp_action::create_server<StringAction>(
      this, "~/action",
      std::bind(&RobotiqController::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&RobotiqController::handle_cancel, this, std::placeholders::_1),
      std::bind(&RobotiqController::handle_accepted, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "RobotIQ controller initialized.");
}

void RobotiqController::update_callback() {
  controller_->read_status();
  auto msg = std_msgs::msg::String();
  msg.data = controller_->status().to_json();
  status_publisher_->publish(msg);
  update_joint_state();
}

void RobotiqController::update_joint_state() {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->get_clock()->now();
  msg.name = {
      "finger_1_joint_1",      "finger_1_joint_2",      "finger_1_joint_3",
      "finger_2_joint_1",      "finger_2_joint_2",      "finger_2_joint_3",
      "finger_middle_joint_1", "finger_middle_joint_2", "finger_middle_joint_3",
      "palm_finger_1_joint",   "palm_finger_2_joint"};

  std::vector<double> joint_positions_request;
  const auto finger_list = controller_->status().fingers();
  for (const auto* finger : finger_list) {
    const int position = finger->position;
    for (int joint = 0; joint < finger->number_of_joints(); ++joint) {
      joint_positions_request.push_back(
          controller_->status().finger_joint_position_from_bit(
              position, joint, finger->scissor));
    }
  }

  msg.position = std::move(joint_positions_request);
  joint_state_publisher_->publish(msg);
}

rclcpp_action::GoalResponse RobotiqController::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const StringAction::Goal> goal) {
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotiqController::handle_cancel(
    const std::shared_ptr<GoalHandleStringAction> goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotiqController::handle_accepted(
    const std::shared_ptr<GoalHandleStringAction> goal_handle) {
  execute(goal_handle);
}

void RobotiqController::execute(
    const std::shared_ptr<GoalHandleStringAction> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<StringAction::Result>();

  if (goal->text == "activate") {
    controller_->send_command();
  } else if (goal->text == "reset") {
    controller_->send_command(false);
  } else if (goal->text == "open") {
    controller_->send_command(true, "basic", true, false, false, false, 0);
  } else if (goal->text == "close") {
    controller_->send_command(true, "basic", true, false, false, false, 255);
  } else if (goal->text == "pinch-open") {
    controller_->send_command(true, "pinch", true, false, false, false, 0);
  } else if (goal->text == "pinch-close") {
    controller_->send_command(true, "pinch", true, false, false, false, 255);
  } else if (goal->text == "wide-open") {
    controller_->send_command(true, "wide", true, false, false, false, 0);
  } else if (goal->text == "wide-close") {
    controller_->send_command(true, "wide", true, false, false, false, 255);
  } else {
    result->success = false;
    result->message = "Unknown command: " + goal->text;
    goal_handle->abort(result);
    return;
  }

  controller_->read_status();

  rclcpp::sleep_for(std::chrono::seconds(1));

  const auto start = std::chrono::steady_clock::now();
  while (controller_->status().motion_status == STATUS_MOTION::IN_MOTION) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - start >
        std::chrono::seconds(timeout_)) {
      result->success = false;
      result->message = "Timeout reached while waiting for action to complete.";
      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }
  }

  if (controller_->status().motion_status ==
      STATUS_MOTION::REACHED_TARGET_POSITION) {
    result->success = true;
    goal_handle->succeed(result);
  } else {
    result->success = false;
    result->message = "Failed to reach target position.";
    goal_handle->abort(result);
  }
}

/**
 * @brief Entry point for the Robotiq controller node.
 * @param argc Number of command-line arguments.
 * @param argv Command-line arguments.
 * @return Process exit code.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotiqController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
