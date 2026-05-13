// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#ifndef ARM_WAVE_DEMO_HPP_
#define ARM_WAVE_DEMO_HPP_

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <vector>

#include "alliander_interfaces/action/trigger_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using TriggerAction = alliander_interfaces::action::TriggerAction;
using GoalHandleTA = rclcpp_action::ClientGoalHandle<TriggerAction>;

enum ArmState {
  Uninitialized = 0,
  Homed = 1,
  GripperOpen = 2,
  GripperClosed = 3,
  Ready = 4,
};

class StateMachine {
 private:
  ArmState state_{ArmState::Uninitialized};
  std::map<ArmState, ArmState> transitions_{
      {ArmState::Uninitialized, ArmState::Homed},
      {ArmState::Homed, ArmState::GripperOpen},
      {ArmState::GripperOpen, ArmState::GripperClosed},
      {ArmState::GripperClosed, ArmState::Ready},
  };

  static std::string get_state_name_(ArmState state) {
    switch (state) {
      case ArmState::Uninitialized:
        return "Uninitialized";
      case ArmState::Homed:
        return "Homed";
      case ArmState::GripperOpen:
        return "Gripper Open";
      case ArmState::GripperClosed:
        return "Gripper Closed";
      default:
        return "Ready";
    }
  }

 public:
  ArmState get_state() { return state_; }

  void reset() { state_ = ArmState::Uninitialized; }

  void transition() {
    ArmState current_state = state_;
    state_ = transitions_[state_];
    if (state_ != current_state) {
      printf("Previous state: %s, current state: %s",
             get_state_name_(current_state).c_str(),
             get_state_name_(state_).c_str());
    }
  }
};

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
  void send_home();

  template <typename ActionT>
  void on_response(
      const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr&
          handle) {
    if (!handle) {
      printf("Goal rejected by controller.");
      busy_ = false;
    } else {
      printf("Goal accepted.");
    }
  }

  template <typename ActionT>
  void on_result(
      const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult&
          result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        printf("Goal complete.");
        state_machine_.transition();
        break;
      case rclcpp_action::ResultCode::ABORTED:
        printf("Goal aborted.");
        state_machine_.reset();
        break;
      case rclcpp_action::ResultCode::CANCELED:
        printf("Goal canceled.");
        state_machine_.reset();
        break;
      default:
        printf("Unknown result code.");
        break;
    }
    busy_ = false;
  }

  /// Action client to send wave motion as a JointTrajectory.
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr fjt_action_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions fjt_send_opts_;
  rclcpp_action::Client<TriggerAction>::SharedPtr gripper_action_client_;
  rclcpp_action::Client<TriggerAction>::SendGoalOptions gripper_send_opts_;
  /// Timer to send motion again when it is finished.
  rclcpp::TimerBase::SharedPtr timer_;

  /// Vector containing robot joint names.
  std::vector<std::string> joints_;
  /// Vector containing neutral joint positions.
  std::vector<double> initial_joint_positions_;
  /// Vector containing indices indicating which joints to move.
  std::vector<int64_t> wave_joint_indices_;
  /// Vector containing amplitudes of movement, in rad.
  std::vector<double> wave_amplitudes_;

  /// Period of up and down movement, in seconds.
  double wave_period_sec_;

  /// Flag indicating whether a movement is in progress.
  std::atomic<bool> busy_{false};

  StateMachine state_machine_;
};

#endif
