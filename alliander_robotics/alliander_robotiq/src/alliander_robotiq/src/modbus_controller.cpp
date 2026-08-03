// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#include "alliander_robotiq/modbus_controller.hpp"

#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "alliander_robotiq/modbus_tcp_client.hpp"

/**
 * @brief Convert protocol position byte to URDF joint angle in radians.
 * @param position Position byte [0..255].
 * @param joint Joint index (1-based).
 * @param scissor True when converting a scissor joint.
 * @return Joint angle in radians.
 */
double finger_joint_position_from_bit(int position, int joint, bool scissor) {
  double lower_limit;
  double upper_limit;

  if (joint == 1) {
    lower_limit = scissor ? -0.1784 : 0.0;
    upper_limit = scissor ? 0.192 : 1.2218;
  } else if (joint == 2) {
    lower_limit = scissor ? -0.192 : 0.0;
    upper_limit = scissor ? 0.1784 : 1.5708;
  } else if (joint == 3) {
    if (scissor) {
      throw std::invalid_argument("Scissor does not have a third joint.");
    }
    lower_limit = 0.0523;
    upper_limit = 1.2217;
  } else {
    throw std::invalid_argument("Unsupported joint number.");
  }

  // Only the first joint of the scissor is inverted:
  const double reach = upper_limit - lower_limit;
  if (scissor && joint == 1) {
    return upper_limit - (static_cast<double>(position) / 255.0) * reach;
  }

  return lower_limit + (static_cast<double>(position) / 255.0) * reach;
}

/**
 * @brief Private implementation of hardware controller transport and status.
 */
class ModbusControllerHardware::Impl {
 public:
  /**
   * @brief Construct implementation object.
   * @param host IPv4 address of target device.
   * @param port Modbus TCP port.
   */
  Impl(const std::string& host, int port) : client(host, port, 5) {}

  /// Low-level Modbus TCP transport.
  ModbusTcpClient client;
  /// Most recently decoded gripper status.
  GripperStatus status;
};

ModbusControllerHardware::ModbusControllerHardware(const std::string& host,
                                                   int port)
    : impl_(std::make_unique<Impl>(host, port)) {}

ModbusControllerHardware::~ModbusControllerHardware() = default;

bool ModbusControllerHardware::open() { return impl_->client.open(); }

void ModbusControllerHardware::send_command(
    bool activate, const std::string& mode, bool go_to, bool automatic_release,
    bool individual_control_fingers, bool individual_control_scissor,
    int position, int speed, int force) {
  if (position < 0 || position > 255 || speed < 0 || speed > 255 || force < 0 ||
      force > 255) {
    throw std::invalid_argument(
        "Position, speed and force must be between 0 and 255.");
  }

  int mode_value = 0;
  if (mode == "basic") {
    mode_value = 0;
  } else if (mode == "pinch") {
    mode_value = 1;
  } else if (mode == "wide") {
    mode_value = 2;
  } else if (mode == "scissor") {
    mode_value = 3;
  } else {
    throw std::invalid_argument("Unsupported mode.");
  }

  // The action byte is defined as [0 0 0 atr gto mod act]:
  auto atr = automatic_release ? 1 : 0, gto = go_to ? 1 : 0, mod = mode_value,
       act = activate ? 1 : 0;
  const std::uint8_t action_byte =
      (static_cast<std::uint8_t>(atr) << 4U) |          // bit 4
      (static_cast<std::uint8_t>(gto) << 3U) |          // bit 3
      (static_cast<std::uint8_t>(mod & 0x03U) << 1U) |  // bit 1&2
      static_cast<std::uint8_t>(act);                   // bit 0

  // The options byte is defined as [0 0 0 0 isf icf 0 0]:
  auto isf = individual_control_scissor ? 1 : 0,
       icf = individual_control_fingers ? 1 : 0;
  const std::uint8_t options_byte =
      (static_cast<std::uint8_t>(isf) << 3U) |  // bit 3
      (static_cast<std::uint8_t>(icf) << 2U);   // bit 2

  // The registers are defined as [action options][empty position][speed force]:
  const std::vector<std::uint16_t> registers = {
      static_cast<std::uint16_t>((action_byte << 8U) | options_byte),
      static_cast<std::uint16_t>(position & 0xFF),
      static_cast<std::uint16_t>(((speed & 0xFF) << 8U) | (force & 0xFF))};

  impl_->client.write_multiple_registers(0, registers);
}

void ModbusControllerHardware::read_status() {
  const auto registers = impl_->client.read_input_registers(0, 8);
  if (registers.size() != 8) {
    return;
  }

  auto gripper_status = (registers[0] >> 8U) & 0xFFU;  // Register 0, high byte
  auto object_status = registers[0] & 0xFFU;           // Register 0, low byte
  auto fault_status = (registers[1] >> 8U) & 0xFFU;    // Register 1, high byte
  auto a_request = registers[1] & 0xFFU;               // Register 1, low byte
  auto a_position = (registers[2] >> 8U) & 0xFFU;      // Register 2, high byte
  auto a_current = registers[2] & 0xFFU;               // Register 2, low byte
  auto b_request = (registers[3] >> 8U) & 0xFFU;       // Register 3, high byte
  auto b_position = registers[3] & 0xFFU;              // Register 3, low byte
  auto b_current = (registers[4] >> 8U) & 0xFFU;       // Register 4, high byte
  auto c_request = registers[4] & 0xFFU;               // Register 4, low byte
  auto c_position = (registers[5] >> 8U) & 0xFFU;      // Register 5, high byte
  auto c_current = registers[5] & 0xFFU;               // Register 5, low byte
  auto s_request = (registers[6] >> 8U) & 0xFFU;       // Register 6, high byte
  auto s_position = registers[6] & 0xFFU;              // Register 6, low byte
  auto s_current = (registers[7] >> 8U) & 0xFFU;       // Register 7, high byte

  impl_->status.update_gripper_status(gripper_status);
  impl_->status.update_finger_status(object_status);
  impl_->status.update_fault_status(fault_status);
  impl_->status.update_finger_states(std::vector<unsigned int>{
      a_request,
      a_position,
      a_current,
      b_request,
      b_position,
      b_current,
      c_request,
      c_position,
      c_current,
      s_request,
      s_position,
      s_current,
  });
}

GripperStatus& ModbusControllerHardware::status() { return impl_->status; }

ModbusControllerSimulation::ModbusControllerSimulation(
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher)
    : publisher_(std::move(publisher)) {
  status_.motion_status = "reached_target_position";
}

bool ModbusControllerSimulation::open() { return true; }

void ModbusControllerSimulation::send_command(
    bool activate, const std::string& mode, bool /*go_to*/,
    bool /*automatic_release*/, bool /*individual_control_fingers*/,
    bool /*individual_control_scissor*/, int position, int /*speed*/,
    int /*force*/) {
  if (!activate) {
    return;
  }

  std::vector<double> joint_positions;
  joint_positions.reserve(11);

  for (int finger = 0; finger < 3; ++finger) {
    (void)finger;
    for (int joint = 1; joint <= 3; ++joint) {
      joint_positions.push_back(
          finger_joint_position_from_bit(position, joint, false));
    }
  }

  int scissor_position = 135;
  if (mode == "pinch") {
    scissor_position = 255;
  } else if (mode == "wide") {
    scissor_position = 0;
  }

  for (int joint = 1; joint <= 2; ++joint) {
    joint_positions.push_back(
        finger_joint_position_from_bit(scissor_position, joint, true));
  }

  std_msgs::msg::Float64MultiArray command;
  command.data = joint_positions;
  publisher_->publish(command);

  status_.mode = mode;
  status_.motion_status = "reached_target_position";
}

void ModbusControllerSimulation::read_status() {}

GripperStatus& ModbusControllerSimulation::status() { return status_; }
