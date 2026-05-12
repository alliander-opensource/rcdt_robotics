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

/// Class to run a "arm-wave" motion on a robot arm.
class ArmWaveDemo : public rclcpp::Node {
 public:
  /**
   * @brief constructor for the ArmWaveDemo class.
   */
  ArmWaveDemo();

 private:
  /**
   * @brief Sends wave motion to action server.
   */
  void send_wave();

  /// Action client to send wave motion as a JointTrajectory.
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  /// Timer to send motion again when it is finished.
  rclcpp::TimerBase::SharedPtr timer_;

  /// Vector containing robot joint names.
  std::vector<std::string> joints_;
  /// Vector containing neutral joint positions.
  std::vector<double> initial_joint_positions_;

  /// Index indicating which joint to move.
  int wave_joint_index_;
  /// Amplitude of movement, in rad.
  double wave_amplitude_;
  /// Period of up and down movement, in seconds.
  double wave_period_sec_;
  /// Flag indicating whether a movement is in progress.
  std::atomic<bool> wave_in_progress_;
};

#endif
