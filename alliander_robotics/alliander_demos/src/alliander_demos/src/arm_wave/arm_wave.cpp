// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "arm_wave/arm_wave.hpp"

ArmWaveDemo::ArmWaveDemo() : Node("arm_wave_demo") {
  this->declare_parameter<std::vector<std::string>>("joints", {
                                                                  "fr3_joint1",
                                                                  "fr3_joint2",
                                                                  "fr3_joint3",
                                                                  "fr3_joint4",
                                                                  "fr3_joint5",
                                                                  "fr3_joint6",
                                                                  "fr3_joint7",
                                                              });
  this->declare_parameter<int>("wave_joint_index", 4);
  this->declare_parameter<double>("wave_amplitude", 0.8);
  this->declare_parameter<double>("wave_period_sec", 8.0);

  joints_ = this->get_parameter("joints").as_string_array();
  wave_joint_index_ = this->get_parameter("wave_joint_index").as_int();
  wave_amplitude_ = this->get_parameter("wave_amplitude").as_double();
  wave_period_sec_ = this->get_parameter("wave_period_sec").as_double();

  action_client_ =
      rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
          this, "wave_controller/follow_joint_trajectory");

  RCLCPP_INFO(get_logger(), "Waiting for action server...");
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(),
                 "Action server not available after 10s. Exiting.");
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(get_logger(), "Action server found. Starting wave.");

  send_wave();
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(wave_period_sec_ * 1000)),
      [this]() {
        if (!wave_in_progress_) {
          send_wave();
        }
      });
}

void ArmWaveDemo::send_wave() {}
