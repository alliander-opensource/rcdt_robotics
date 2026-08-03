// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#include "alliander_robotiq/gripper_status.hpp"

#include <array>
#include <sstream>

#include "alliander_robotiq/json_utils.hpp"

GripperStatus::GripperStatus() { finger_s.scissor = true; }

std::array<FingerStatus*, 4> GripperStatus::fingers() {
  return {&finger_a, &finger_b, &finger_c, &finger_s};
}

std::array<const FingerStatus*, 4> GripperStatus::fingers() const {
  return {&finger_a, &finger_b, &finger_c, &finger_s};
}

void GripperStatus::update_gripper_status(std::uint8_t byte) {
  static const std::array<std::string, 4> modes = {"basic", "pinch", "wide",
                                                   "scissor"};
  static const std::array<std::string, 4> statuses = {
      "resetting", "activation_in_progress", "mode_change_in_progress",
      "changes_complete"};
  static const std::array<std::string, 4> motion_states = {
      "in_motion", "not_all_fingers_reached_target_position",
      "no_fingers_reached_target_position", "reached_target_position"};

  active = (byte & 0x01U) != 0U;
  mode = modes[(byte >> 1U) & 0x03U];
  go_to = ((byte >> 3U) & 0x01U) != 0U;
  gripper_status = statuses[(byte >> 4U) & 0x03U];
  motion_status = motion_states[(byte >> 6U) & 0x03U];
}

void GripperStatus::update_finger_status(std::uint8_t byte) {
  finger_a.update_status((byte >> 0U) & 0x03U);
  finger_b.update_status((byte >> 2U) & 0x03U);
  finger_c.update_status((byte >> 4U) & 0x03U);
  finger_s.update_status((byte >> 6U) & 0x03U);
}

void GripperStatus::update_fault_status(std::uint8_t byte) {
  static const std::array<std::string, 10> fault_statuses = {
      "no_fault",
      "activation_must_be_completed",
      "mode_change_must_be_completed",
      "activation_bit_must_be_set",
      "communication_chip_not_ready",
      "interference_on_scissor",
      "automatic_release_in_progress",
      "activation_fault_verify_interference",
      "changing_mode_fault_verify_interference",
      "automatic_release_completed_reset_required"};

  const std::uint8_t status_code = byte & 0x0FU;
  switch (status_code) {
    case 0:
      fault_status = fault_statuses[0];
      break;
    case 5:
      fault_status = fault_statuses[1];
      break;
    case 6:
      fault_status = fault_statuses[2];
      break;
    case 7:
      fault_status = fault_statuses[3];
      break;
    case 9:
      fault_status = fault_statuses[4];
      break;
    case 10:
      fault_status = fault_statuses[5];
      break;
    case 11:
      fault_status = fault_statuses[6];
      break;
    case 12:
      fault_status = fault_statuses[7];
      break;
    case 13:
      fault_status = fault_statuses[8];
      break;
    case 14:
      fault_status = fault_statuses[9];
      break;
    default:
      fault_status = "no_fault";
      break;
  }
}

void GripperStatus::update_finger_states(
    const std::vector<unsigned int>& states) {
  auto finger_list = fingers();
  for (std::size_t idx = 0; idx < finger_list.size(); ++idx) {
    auto* finger = finger_list[idx];
    finger->position_request = static_cast<int>(states[idx * 3]);
    finger->position = static_cast<int>(states[idx * 3 + 1]);
    finger->current = static_cast<int>(states[idx * 3 + 2]);
  }
}

std::string GripperStatus::to_json() const {
  std::ostringstream out;
  out << "{";
  out << "\"active\":" << (active ? "true" : "false") << ",";
  out << "\"mode\":" << json_string(mode) << ",";
  out << "\"go_to\":" << (go_to ? "true" : "false") << ",";
  out << "\"gripper_status\":" << json_string(gripper_status) << ",";
  out << "\"motion_status\":" << json_string(motion_status) << ",";
  out << "\"fault_status\":" << json_string(fault_status) << ",";
  out << "\"finger_a\":" << finger_a.to_json() << ",";
  out << "\"finger_b\":" << finger_b.to_json() << ",";
  out << "\"finger_c\":" << finger_c.to_json() << ",";
  out << "\"finger_s\":" << finger_s.to_json();
  out << "}";
  return out.str();
}
