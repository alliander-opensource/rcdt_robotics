// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#include "alliander_robotiq/finger_status.hpp"

#include <array>
#include <sstream>
#include <stdexcept>

#include "alliander_robotiq/json_utils.hpp"

int FingerStatus::number_of_joints() const { return scissor ? 2 : 3; }

void FingerStatus::update_status(std::uint8_t status_index) {
  static const std::array<std::string, 4> statuses = {
      "in_motion", "contact_while_opening", "contact_while_closing",
      "reached_target_position"};

  if (status_index >= statuses.size()) {
    throw std::out_of_range("Invalid finger status index.");
  }

  status = statuses[status_index];
}

std::string FingerStatus::to_json() const {
  std::ostringstream out;
  out << "{";
  out << "\"scissor\":" << (scissor ? "true" : "false") << ",";
  out << "\"status\":" << json_string(status) << ",";
  out << "\"position_request\":" << position_request << ",";
  out << "\"position\":" << position << ",";
  out << "\"current\":" << current;
  out << "}";
  return out.str();
}
