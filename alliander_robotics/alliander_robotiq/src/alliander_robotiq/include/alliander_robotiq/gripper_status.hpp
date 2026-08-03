// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ALLIANDER_ROBOTIQ__GRIPPER_STATUS_HPP_
#define ALLIANDER_ROBOTIQ__GRIPPER_STATUS_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "alliander_robotiq/finger_status.hpp"

/**
 * @brief Aggregated status of the Robotiq gripper and all fingers.
 */
struct GripperStatus {
  /// Activation bit from the gripper status byte.
  bool active = false;
  /// Active grasping mode label.
  std::string mode = "basic";
  /// Go-to flag indicating motion command execution.
  bool go_to = false;
  /// Global gripper lifecycle state label.
  std::string gripper_status = "resetting";
  /// Global motion state label.
  std::string motion_status = "in_motion";
  /// Current fault state label.
  std::string fault_status = "no_fault";
  /// Finger A status.
  FingerStatus finger_a;
  /// Finger B status.
  FingerStatus finger_b;
  /// Finger C status.
  FingerStatus finger_c;
  /// Scissor status.
  FingerStatus finger_s;

  /**
   * @brief Construct with defaults and mark finger_s as scissor.
   */
  GripperStatus();

  /**
   * @brief Mutable list view of all fingers.
   * @return Ordered pointers [A, B, C, S].
   */
  std::array<FingerStatus*, 4> fingers();

  /**
   * @brief Const list view of all fingers.
   * @return Ordered pointers [A, B, C, S].
   */
  std::array<const FingerStatus*, 4> fingers() const;

  /**
   * @brief Decode and update global gripper fields from protocol byte.
   * @param byte Status byte from the gripper protocol.
   */
  void update_gripper_status(std::uint8_t byte);

  /**
   * @brief Decode and update finger contact/motion states from protocol byte.
   * @param byte Object status byte from the gripper protocol.
   */
  void update_finger_status(std::uint8_t byte);

  /**
   * @brief Decode and update fault status from protocol byte.
   * @param byte Fault status byte from the gripper protocol.
   */
  void update_fault_status(std::uint8_t byte);

  /**
   * @brief Update position/current bytes for all fingers.
   * @param states Array of 12 bytes grouped per finger
   * [position_request, position, current].
   */
  void update_finger_states(const std::vector<unsigned int>& states);

  /**
   * @brief Serialize this structure to a JSON string.
   * @return JSON representation of gripper and finger state.
   */
  std::string to_json() const;
};

#endif  // ALLIANDER_ROBOTIQ__GRIPPER_STATUS_HPP_
