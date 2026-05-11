// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#ifndef ARM_WAVE_DEMO_HPP_
#define ARM_WAVE_DEMO_HPP_

#include <cmath>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class ArmWaveDemo : public rclcpp::Node {
 public:
  ArmWaveDemo();

 private:
  void send_wave();

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joints_;

  int wave_joint_index_;
  double wave_amplitude_;
  double wave_period_sec_;
  std::atomic<bool> wave_in_progress_;
};

#endif
