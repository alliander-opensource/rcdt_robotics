// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#include "alliander_robotiq/json_utils.hpp"

#include <sstream>

std::string json_string(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const char ch : value) {
    if (ch == '\\') {
      out << "\\\\";
    } else if (ch == '"') {
      out << "\\\"";
    } else {
      out << ch;
    }
  }
  out << '"';
  return out.str();
}
