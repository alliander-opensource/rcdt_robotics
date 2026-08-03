// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ALLIANDER_ROBOTIQ__FINGER_STATUS_HPP_
#define ALLIANDER_ROBOTIQ__FINGER_STATUS_HPP_

#include <cstdint>
#include <string>

/**
 * @brief Holds runtime status values of a single Robotiq finger.
 */
struct FingerStatus {
  /// True when this finger represents the scissor axis.
  bool scissor = false;
  /// Status label from the gripper protocol.
  std::string status = "in_motion";
  /// Requested position byte [0..255].
  int position_request = 0;
  /// Measured position byte [0..255].
  int position = 0;
  /// Measured motor current byte.
  int current = 0;

  /**
   * @brief Get the number of joints represented by this finger.
   * @return 2 for scissor, otherwise 3.
   */
  int number_of_joints() const;

  /**
   * @brief Update the protocol status label from a status index.
   * @param status_index Status index in range [0..3].
   */
  void update_status(std::uint8_t status_index);

  /**
   * @brief Serialize this structure to a JSON string.
   * @return JSON representation of finger state.
   */
  std::string to_json() const;
};

#endif  // ALLIANDER_ROBOTIQ__FINGER_STATUS_HPP_
