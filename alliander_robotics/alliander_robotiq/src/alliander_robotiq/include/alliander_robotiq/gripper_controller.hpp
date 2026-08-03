// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ALLIANDER_ROBOTIQ__GRIPPER_CONTROLLER_HPP_
#define ALLIANDER_ROBOTIQ__GRIPPER_CONTROLLER_HPP_

#include <alliander_interfaces/action/string_action.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "alliander_robotiq/modbus_controller.hpp"

typedef alliander_interfaces::action::StringAction StringAction;
typedef rclcpp_action::ServerGoalHandle<StringAction> GoalHandleStringAction;
typedef std_msgs::msg::Float64MultiArray Float64MultiArray;
typedef std_msgs::msg::String String;
typedef sensor_msgs::msg::JointState JointState;

/// Node to control the Robotiq 3-finger gripper.
class RobotiqController : public rclcpp::Node {
 public:
  /**
   * @brief constructor for the RobotiqController class.
   */
  RobotiqController();

 private:
  /// Publisher of serialized gripper status.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  /// Publisher of kinematic joint states.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  /// Publisher used in simulation mode to command joint controller.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      simulation_controller_publisher_;
  /// Timer used to update status.
  rclcpp::TimerBase::SharedPtr timer_;
  /// Action server accepting text commands.
  rclcpp_action::Server<StringAction>::SharedPtr action_server_;
  /// Active hardware or simulation controller implementation.
  std::unique_ptr<IModbusController> controller_;

  /// Update rate.
  static constexpr std::chrono::milliseconds update_rate_{100};
  /// Action timeout in seconds.
  static constexpr int timeout_ = 15;

  /**
   * @brief Poll hardware status and publish status/joint-state topics.
   */
  void update_callback();

  /**
   * @brief Convert current finger bytes to joint state message and publish.
   */
  void update_joint_state();

  /**
   * @brief Accept incoming action goals.
   * @param uuid Goal UUID.
   * @param goal Goal payload.
   * @return ACCEPT_AND_EXECUTE for all goals.
   */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const StringAction::Goal> goal);

  /**
   * @brief Accept cancellation requests.
   * @param goal_handle Goal to cancel.
   * @return ACCEPT always.
   */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleStringAction> goal_handle);

  /**
   * @brief Execute accepted goals.
   * @param goal_handle Accepted goal handle.
   */
  void handle_accepted(
      const std::shared_ptr<GoalHandleStringAction> goal_handle);

  /**
   * @brief Execute a gripper command and complete the action goal.
   * @param goal_handle Goal to execute.
   */
  void execute(const std::shared_ptr<GoalHandleStringAction> goal_handle);
};

#endif  // ALLIANDER_ROBOTIQ__GRIPPER_CONTROLLER_HPP_
