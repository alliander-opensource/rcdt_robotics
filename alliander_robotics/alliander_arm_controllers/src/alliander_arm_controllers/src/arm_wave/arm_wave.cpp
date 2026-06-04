// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "arm_wave/arm_wave.hpp"

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

ArmWaveDemo::ArmWaveDemo() : Node("arm_wave_demo") {
  // Joint names and positions are Franka FR3 home configuration
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

  this->declare_parameter<std::vector<int>>("wave_joint_indices", {3});
  this->declare_parameter<std::vector<float>>("wave_amplitudes", {0.5});
  this->declare_parameter<bool>("control_gripper", true);
  this->declare_parameter<double>("wave_period_sec", 8.0);
  this->declare_parameter<double>("wait_time_sec", 3.0);

  joints_ = this->get_parameter("joints").as_string_array();
  initial_joint_positions_ =
      this->get_parameter("initial_joint_positions").as_double_array();
  wave_joint_indices_ =
      this->get_parameter("wave_joint_indices").as_integer_array();
  wave_amplitudes_ = this->get_parameter("wave_amplitudes").as_double_array();
  control_gripper_ = this->get_parameter("control_gripper").as_bool();
  wave_period_sec_ = this->get_parameter("wave_period_sec").as_double();
  wait_time_msec_ =
      (int)(1000 * this->get_parameter("wait_time_sec").as_double());

  fjt_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "joint_trajectory_controller/follow_joint_trajectory");
  RCLCPP_INFO(get_logger(),
              "Waiting for FollowJointTrajectory action server...");
  if (!fjt_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(get_logger(),
                 "FollowJointTrajectory action server not available after 10s. "
                 "Exiting.");
    rclcpp::shutdown();
    return;
  }

  if (control_gripper_) {
    open_gripper_action_client_ =
        rclcpp_action::create_client<GripperAction>(this, "gripper/open");
    RCLCPP_INFO(get_logger(), "Waiting for Open Gripper action server...");
    if (!open_gripper_action_client_->wait_for_action_server(
            std::chrono::seconds(10))) {
      RCLCPP_ERROR(
          get_logger(),
          "Open gripper action server not available after 10s. Exiting.");
      rclcpp::shutdown();
      return;
    }

    close_gripper_action_client_ =
        rclcpp_action::create_client<GripperAction>(this, "gripper/close");
    RCLCPP_INFO(get_logger(), "Waiting for Close Gripper action server...");
    if (!close_gripper_action_client_->wait_for_action_server(
            std::chrono::seconds(10))) {
      RCLCPP_ERROR(
          get_logger(),
          "Close gripper action server not available after 10s. Exiting.");
      rclcpp::shutdown();
      return;
    }
  }

  RCLCPP_INFO(get_logger(), "Action servers found. Starting wave.");

  fjt_send_opts_.goal_response_callback =
      [this](const GoalHandleFJT::SharedPtr& handle) {
        on_response<FollowJointTrajectory>(handle);
      };

  fjt_send_opts_.result_callback =
      [this](const GoalHandleFJT::WrappedResult& result) {
        on_result<FollowJointTrajectory>(result);
      };

  gripper_send_opts_.goal_response_callback =
      [this](const GoalHandleGripper::SharedPtr& handle) {
        on_response<GripperAction>(handle);
      };

  gripper_send_opts_.result_callback =
      [this](const GoalHandleGripper::WrappedResult& result) {
        on_result<GripperAction>(result);
      };

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(100)), [this]() {
        if (!busy_) {
          switch (state_machine_.get_state()) {
            case ArmState::Uninitialized:
              send_home();
              break;
            case ArmState::Homed:
              control_gripper_ ? open_gripper() : state_machine_.transition();
              break;
            case ArmState::GripperOpen:
              control_gripper_ ? close_gripper() : state_machine_.transition();
              break;
            case ArmState::GripperClosed:
              get_ready();
              break;
            case ArmState::Ready:
              send_wave();
              break;
          }
        }
      });
}

void ArmWaveDemo::send_home() {
  RCLCPP_INFO(get_logger(), "Sending home command to arm.");
  busy_ = true;

  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory.joint_names = joints_;

  trajectory_msgs::msg::JointTrajectoryPoint home;
  home.positions = initial_joint_positions_;
  home.time_from_start.sec = 4;

  goal.trajectory.points = {home};

  fjt_action_client_->async_send_goal(goal, fjt_send_opts_);
}

void ArmWaveDemo::open_gripper() {
  RCLCPP_INFO(get_logger(), "Sending open gripper command to arm.");
  busy_ = true;

  auto goal = GripperAction::Goal();
  open_gripper_action_client_->async_send_goal(goal, gripper_send_opts_);
}

void ArmWaveDemo::close_gripper() {
  RCLCPP_INFO(get_logger(),
              "Waiting for a few seconds before closing gripper.");
  busy_ = true;

  rclcpp::sleep_for(std::chrono::milliseconds(wait_time_msec_));
  RCLCPP_INFO(get_logger(), "Sending close gripper command.");

  auto goal = GripperAction::Goal();
  close_gripper_action_client_->async_send_goal(goal, gripper_send_opts_);
}

void ArmWaveDemo::get_ready() {
  RCLCPP_INFO(get_logger(), "Waiting for a few seconds before starting wave.");
  rclcpp::sleep_for(std::chrono::milliseconds(wait_time_msec_));
  state_machine_.transition();
}

void ArmWaveDemo::send_wave() {
  busy_ = true;

  auto goal = FollowJointTrajectory::Goal();
  goal.trajectory.joint_names = joints_;

  const int n = static_cast<int>(joints_.size());

  trajectory_msgs::msg::JointTrajectoryPoint up, down;
  up.positions = initial_joint_positions_;
  down.positions = initial_joint_positions_;
  for (int i = 0; i < wave_joint_indices_.size(); i++) {
    int joint_index = wave_joint_indices_[i];
    float amplitude = wave_amplitudes_[i];
    if (i < n) {
      up.positions[joint_index] += amplitude;
      down.positions[joint_index] -= amplitude;
    }
  }

  float half_sec = wave_period_sec_ / 2.0;
  up.time_from_start.sec = (int)half_sec;
  up.time_from_start.nanosec = (int)((half_sec - (int)half_sec) * 1e9);

  down.time_from_start.sec = static_cast<int>(wave_period_sec_);
  down.time_from_start.nanosec = 0;

  goal.trajectory.points = {up, down};

  fjt_action_client_->async_send_goal(goal, fjt_send_opts_);
}
