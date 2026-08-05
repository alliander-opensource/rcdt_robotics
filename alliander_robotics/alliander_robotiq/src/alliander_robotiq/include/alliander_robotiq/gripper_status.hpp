// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ALLIANDER_ROBOTIQ__GRIPPER_STATUS_HPP_
#define ALLIANDER_ROBOTIQ__GRIPPER_STATUS_HPP_

#include <cstdint>
#include <glaze/glaze.hpp>
#include <magic_enum/magic_enum.hpp>
#include <string>
#include <vector>

#include "alliander_robotiq/finger_status.hpp"

// Use magic_enum to serialize enum values to JSON:
// https://stephenberry.github.io/glaze/enum-reflection/
template <typename T>
  requires std::is_enum_v<T>
struct glz::meta<T> {
  static constexpr auto keys = magic_enum::enum_names<T>();
  static constexpr auto value = magic_enum::enum_values<T>();
};

// Definitions:
enum class MODES {
  BASIC,
  PINCH,
  WIDE,
  SCISSOR,
};
enum class STATUS_GRIPPER {
  RESETTING,
  ACTIVATION_IN_PROGRESS,
  MODE_CHANGE_IN_PROGRESS,
  CHANGES_COMPLETE
};
enum class STATUS_MOTION {
  IN_MOTION,
  NOT_ALL_FINGERS_REACHED_TARGET_POSITION,
  NO_FINGERS_REACHED_TARGET_POSITION,
  REACHED_TARGET_POSITION
};
enum class STATUS_FAULT {
  NO_FAULT,
  ACTIVATION_MUST_BE_COMPLETED,
  MODE_CHANGE_MUST_BE_COMPLETED,
  ACTIVATION_BIT_MUST_BE_SET,
  COMMUNICATION_CHIP_NOT_READY,
  INTERFERENCE_ON_SCISSOR,
  AUTOMATIC_RELEASE_IN_PROGRESS,
  ACTIVATION_FAULT_VERIFY_INTERFERENCE,
  CHANGING_MODE_FAULT_VERIFY_INTERFERENCE,
  AUTOMATIC_RELEASE_COMPLETED_RESET_REQUIRED
};

// Joint limits as defined in the URDF:
struct JointLimit {
  double lower_limit;
  double upper_limit;
};

struct JointLimits {
  std::vector<JointLimit> joints{JointLimit{0.0, 1.2218},
                                 JointLimit{0.0, 1.5708},
                                 JointLimit{0.0523, 1.2217}};
  std::vector<JointLimit> scissors{JointLimit{-0.1784, 0.192},
                                   JointLimit{-0.192, 0.1784}};
};

/**
 * @brief Aggregated status of the Robotiq gripper and all fingers.
 */
struct GripperStatus {
  /// Activation bit from the gripper status byte.
  bool active;
  /// Active grasping mode label.
  MODES mode;
  /// Go-to flag indicating motion command execution.
  bool go_to;
  /// Global gripper lifecycle state label.
  STATUS_GRIPPER gripper_status;
  /// Global motion state label.
  STATUS_MOTION motion_status;
  /// Current fault state label.
  STATUS_FAULT fault_status;
  /// Finger A status.
  FingerStatus finger_a;
  /// Finger B status.
  FingerStatus finger_b;
  /// Finger C status.
  FingerStatus finger_c;
  /// Scissor status.
  FingerStatus finger_s{true};

  std::vector<FingerStatus*> fingers() {
    return {&finger_a, &finger_b, &finger_c, &finger_s};
  };

  /**
   * @brief Decode and update global gripper fields from protocol byte.
   * @param byte Status byte from the gripper protocol.
   */
  void update_gripper_status(std::uint8_t byte) {
    active = (byte & 0x01U);                                // bit 0
    mode = MODES((byte >> 1U) & 0x03U);                     // bit 1&2
    go_to = ((byte >> 3U) & 0x01U) != 0U;                   // bit 3
    gripper_status = STATUS_GRIPPER((byte >> 4U) & 0x03U);  // bit 4&5
    motion_status = STATUS_MOTION((byte >> 6U) & 0x03U);    // bit 6&7
  }

  /**
   * @brief Decode and update finger contact/motion states from protocol byte.
   * @param byte Object status byte from the gripper protocol.
   */
  void update_finger_status(std::uint8_t byte) {
    for (auto* finger : fingers()) {
      finger->update_status(
          static_cast<STATUS_FINGER>(byte & 0x03U));  // 2 bits
      byte >>= 2U;  // Shift to the next 2 bits
    }
  }

  /**
   * @brief Decode and update fault status from protocol byte.
   * @param byte Fault status byte from the gripper protocol.
   */
  void update_fault_status(std::uint8_t byte) {
    fault_status = static_cast<STATUS_FAULT>(byte & 0x0FU);  // bit 0-3
  }

  /**
   * @brief Update position/current bytes for all fingers.
   * @param bytes Array of 12 bytes grouped per finger
   * [position_request, position, current] for each finger.
   */
  void update_finger_states(const std::vector<uint8_t>& bytes) {
    int index = 0;
    for (auto* finger : fingers()) {
      finger->position_request = bytes[index * 3];
      finger->position = bytes[index * 3 + 1];
      finger->current = bytes[index * 3 + 2];
      ++index;
    }
  }

  double finger_joint_position_from_bit(int position, int joint, bool scissor) {
    auto upper_limit = scissor ? JointLimits().scissors[joint].upper_limit
                               : JointLimits().joints[joint - 1].upper_limit;
    auto lower_limit = scissor ? JointLimits().scissors[joint].lower_limit
                               : JointLimits().joints[joint - 1].lower_limit;
    const double reach = upper_limit - lower_limit;

    // Only the first joint of the scissor is inverted:
    if (scissor && joint == 1) {
      return upper_limit - (static_cast<double>(position) / 255.0) * reach;
    }

    return lower_limit + (static_cast<double>(position) / 255.0) * reach;
  }

  /**
   * @brief Serialize this structure to a JSON string.
   * @return JSON representation of gripper and finger state.
   */
  std::string to_json() const {
    std::string buffer = glz::write_json(this).value();
    return buffer;
  }
};

#endif  // ALLIANDER_ROBOTIQ__GRIPPER_STATUS_HPP_
