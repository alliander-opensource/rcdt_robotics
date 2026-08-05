// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#include "alliander_robotiq/modbus_controller.hpp"

#include <modbus/modbus.h>

#include <array>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

class ModbusControllerHardware::Impl {
 public:
  Impl(const std::string& host, int port)
      : ctx_(modbus_new_tcp(host.c_str(), port)) {}

  ~Impl() {
    if (ctx_) {
      modbus_close(ctx_);
      modbus_free(ctx_);
    }
  }

  /// libmodbus context owning the TCP connection.
  modbus_t* ctx_;
  /// Most recently decoded gripper status.
  GripperStatus status;
};

ModbusControllerHardware::ModbusControllerHardware(const std::string& host,
                                                   int port)
    : impl_(std::make_unique<Impl>(host, port)) {}

ModbusControllerHardware::~ModbusControllerHardware() = default;

bool ModbusControllerHardware::open() {
  if (!impl_->ctx_) {
    return false;
  }
  modbus_set_response_timeout(impl_->ctx_, 5, 0);
  return modbus_connect(impl_->ctx_) != -1;
}

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

  modbus_write_registers(impl_->ctx_, 0, static_cast<int>(registers.size()),
                         registers.data());
}

void ModbusControllerHardware::read_status() {
  std::array<std::uint16_t, 8> registers{};
  if (modbus_read_input_registers(impl_->ctx_, 0, 8, registers.data()) != 8) {
    return;
  }

  // Split the registers into bytes:
  std::vector<uint8_t> bytes;
  for (const auto reg : registers) {
    bytes.push_back(static_cast<uint8_t>((reg >> 8U) & 0xFFU));  // High byte
    bytes.push_back(static_cast<uint8_t>(reg & 0xFFU));          // Low byte
  }

  // Update the status:
  impl_->status.update_gripper_status(bytes[0]);
  impl_->status.update_finger_status(bytes[1]);
  impl_->status.update_fault_status(bytes[2]);
  impl_->status.update_finger_states(
      std::vector<uint8_t>(bytes.begin() + 3, bytes.end()));
}

GripperStatus& ModbusControllerHardware::status() { return impl_->status; }

ModbusControllerSimulation::ModbusControllerSimulation(
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher)
    : publisher_(std::move(publisher)) {
  gripper_status_.motion_status = STATUS_MOTION::IN_MOTION;
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
      joint_positions.push_back(gripper_status_.finger_joint_position_from_bit(
          position, joint, false));
    }
  }

  int scissor_position = 135;
  if (mode == "pinch") {
    scissor_position = 255;
  } else if (mode == "wide") {
    scissor_position = 0;
  }

  for (int joint = 1; joint <= 2; ++joint) {
    joint_positions.push_back(gripper_status_.finger_joint_position_from_bit(
        scissor_position, joint, true));
  }

  std_msgs::msg::Float64MultiArray command;
  command.data = joint_positions;
  publisher_->publish(command);
}

void ModbusControllerSimulation::read_status() {}

GripperStatus& ModbusControllerSimulation::status() { return gripper_status_; }
