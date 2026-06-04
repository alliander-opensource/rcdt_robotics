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
using GripperAction = alliander_interfaces::action::TriggerAction;
using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperAction>;

/// Enum listing possible states for the arm wave demo.
enum ArmState {
  Uninitialized = 0,
  Homed = 1,
  GripperOpen = 2,
  GripperClosed = 3,
  Ready = 4,
};

/// Class that handles state management and transitions for the arm wave demo.
class StateMachine {
 private:
  /// Current state.
  ArmState state_{ArmState::Uninitialized};
  /// Map of state transitions, to be used in transition().
  std::map<ArmState, ArmState> transitions_{
      {ArmState::Uninitialized, ArmState::Homed},
      {ArmState::Homed, ArmState::GripperOpen},
      {ArmState::GripperOpen, ArmState::GripperClosed},
      {ArmState::GripperClosed, ArmState::Ready},
      {ArmState::Ready, ArmState::Ready},
  };

  /**
   * @brief Returns a human-readable state name.
   * @param state
   * @return String representing current state.
   */
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
  /**
   * @brief Returns the internal state.
   * @return Enum containing the state.
   */
  ArmState get_state() { return state_; }

  /**
   * @brief Resets the state to Uninitialized, starting over.
   */
  void reset() { state_ = ArmState::Uninitialized; }

  /**
   * @brief Transitions to the next state.
   */
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
   * @brief Sends arm to home/middle position.
   */
  void send_home();
  /**
   * @brief Opens the gripper.
   */
  void open_gripper();
  /**
   * @brief Closes the gripper.
   */
  void close_gripper();
  /**
   * @brief Waits for a bit before starting the wave.
   */
  void get_ready();
  /**
   * @brief Sends wave motion to action server.
   */
  void send_wave();

  /**
   * @brief Templated function that handles an action server response.
   * @param handle
   */
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

  /**
   * @brief Templated function that handles an action result.
   * @param result
   */
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
  /// Joint trajectory send options using the templated callback functions.
  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions fjt_send_opts_;
  /// Action client to open the gripper.
  rclcpp_action::Client<GripperAction>::SharedPtr open_gripper_action_client_;
  /// Action client to close the gripper.
  rclcpp_action::Client<GripperAction>::SharedPtr close_gripper_action_client_;
  /// Gripper action client send options using the templated callback functions.
  rclcpp_action::Client<GripperAction>::SendGoalOptions gripper_send_opts_;
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

  /// Boolean indicating whether to control the gripper.
  bool control_gripper_;
  /// Period of up and down movement, in seconds.
  double wave_period_sec_;
  /// Wait time between opening/closing and waving, in milliseconds.
  int wait_time_msec_;

  /// Flag indicating whether a movement is in progress.
  std::atomic<bool> busy_{false};

  /// State machine to handle homing, opening/closing the gripper, and waving.
  StateMachine state_machine_;
};

#endif
