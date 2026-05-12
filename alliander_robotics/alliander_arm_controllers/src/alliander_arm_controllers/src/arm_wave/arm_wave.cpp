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
  this->declare_parameter<std::vector<float>>(
      "initial_joint_positions",
      {0., -0.785398, 0., -2.56194, 0., 1.570796, 0.785398});
  this->declare_parameter<int>("wave_joint_index", 3);
  this->declare_parameter<double>("wave_amplitude", 0.5);
  this->declare_parameter<double>("wave_period_sec", 8.0);

  joints_ = this->get_parameter("joints").as_string_array();
  initial_joint_positions_ =
      this->get_parameter("initial_joint_positions").as_double_array();
  wave_joint_index_ = this->get_parameter("wave_joint_index").as_int();
  wave_amplitude_ = this->get_parameter("wave_amplitude").as_double();
  wave_period_sec_ = this->get_parameter("wave_period_sec").as_double();

  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
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
      std::chrono::milliseconds(static_cast<int>(200)), [this]() {
        if (!wave_in_progress_) {
          send_wave();
        }
      });
}

void ArmWaveDemo::send_wave() {
  wave_in_progress_ = true;

  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory.joint_names = joints_;

  const int n = static_cast<int>(joints_.size());

  // Go up
  trajectory_msgs::msg::JointTrajectoryPoint up;
  up.positions = initial_joint_positions_;
  if (wave_joint_index_ < n) {
    up.positions[wave_joint_index_] += wave_amplitude_;
  }

  float half_sec = wave_period_sec_ / 2.0;
  up.time_from_start.sec = (int)half_sec;
  up.time_from_start.nanosec = (int)((half_sec - (int)half_sec) * 1e9);

  // And back down
  trajectory_msgs::msg::JointTrajectoryPoint down;
  down.positions = initial_joint_positions_;
  if (wave_joint_index_ < n) {
    down.positions[wave_joint_index_] -= wave_amplitude_;
  }
  down.time_from_start.sec = static_cast<int>(wave_period_sec_);
  down.time_from_start.nanosec = 0;

  goal.trajectory.points = {up, down};

  auto send_opts =
      rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

  send_opts.goal_response_callback =
      [this](const GoalHandleFJT::SharedPtr& handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Goal rejected by controller.");
          wave_in_progress_ = false;
        } else {
          RCLCPP_DEBUG(get_logger(), "Goal accepted.");
        }
      };

  send_opts.result_callback =
      [this](const GoalHandleFJT::WrappedResult& result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_DEBUG(get_logger(), "Wave complete.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Goal aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Goal canceled.");
            break;
          default:
            RCLCPP_WARN(get_logger(), "Unknown result code.");
            break;
        }
        wave_in_progress_ = false;
      };

  action_client_->async_send_goal(goal, send_opts);
}
