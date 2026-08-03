// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ALLIANDER_ROBOTIQ__JSON_UTILS_HPP_
#define ALLIANDER_ROBOTIQ__JSON_UTILS_HPP_

#include <string>

/**
 * @brief Escape and quote a string for safe embedding in JSON output.
 * @param value Raw string value.
 * @return JSON-escaped string surrounded by double quotes.
 */
std::string json_string(const std::string& value);

#endif  // ALLIANDER_ROBOTIQ__JSON_UTILS_HPP_
