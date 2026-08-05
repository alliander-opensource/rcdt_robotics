// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ALLIANDER_ROBOTIQ__FINGER_STATUS_HPP_
#define ALLIANDER_ROBOTIQ__FINGER_STATUS_HPP_

enum class STATUS_FINGER {
  IN_MOTION,
  CONTACT_WHILE_OPENING,
  CONTACT_WHILE_CLOSING,
  REACHED_TARGET_POSITION
};

/**
 * @brief Holds the status of a specific finger.
 */
struct FingerStatus {
  /// True when this finger represents the scissor axis.
  bool scissor = false;
  /// Status label from the gripper protocol.
  STATUS_FINGER status;
  /// Requested position byte [0..255].
  int position_request;
  /// Measured position byte [0..255].
  int position;
  /// Measured motor current byte.
  int current;

  /**
   * @brief Get the number of joints represented by this finger.
   * @return 2 for scissor, otherwise 3.
   */
  int number_of_joints() const { return scissor ? 2 : 3; }

  /**
   * @brief Update the protocol status label from a status index.
   * @param status_index Status index in range [0..3].
   */
  void update_status(STATUS_FINGER status) { this->status = status; }
};

#endif  // ALLIANDER_ROBOTIQ__FINGER_STATUS_HPP_
