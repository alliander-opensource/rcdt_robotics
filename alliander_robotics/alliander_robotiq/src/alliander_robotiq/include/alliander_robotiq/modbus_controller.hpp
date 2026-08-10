// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ALLIANDER_ROBOTIQ__MODBUS_CONTROLLER_HPP_
#define ALLIANDER_ROBOTIQ__MODBUS_CONTROLLER_HPP_

#include <modbus/modbus.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

#include "alliander_robotiq/gripper_status.hpp"

// ==========================
// ModbusController Interface
// ==========================

/**
 * @brief Interface for hardware and simulation Modbus controllers.
 */
class IModbusController {
 public:
  virtual ~IModbusController() = default;

  /// The current status of the gripper.
  GripperStatus gripper_status_;

  /**
   * @brief Send a command to the gripper.
   * @param activate Activate/reset bit.
   * @param mode Gripper mode label.
   * @param go_to Execute movement towards requested position.
   * @param automatic_release Trigger automatic release mode.
   * @param individual_control_fingers Enable per-finger control bits.
   * @param individual_control_scissor Enable scissor control bit.
   * @param position Position byte [0..255].
   * @param speed Speed byte [0..255].
   * @param force Force byte [0..255].
   */
  virtual void send_command(bool activate = true,
                            const std::string& mode = "basic",
                            bool go_to = true, bool automatic_release = false,
                            bool individual_control_fingers = false,
                            bool individual_control_scissor = false,
                            int position = 0, int speed = 0, int force = 0) = 0;

  /**
   * @brief Refresh status fields from underlying transport.
   */
  virtual void read_status() = 0;
};

// =======================================
// ModbusControllerHardware implementation
// =======================================

/**
 * @brief Modbus TCP controller for real Robotiq hardware.
 */
class ModbusControllerHardware : public IModbusController {
 public:
  /**
   * @brief Construct hardware controller.
   * @param host IPv4 address of the gripper controller.
   * @param port Modbus TCP port.
   */
  ModbusControllerHardware(const std::string& host, int port);
  ~ModbusControllerHardware();
  void send_command(bool activate = true, const std::string& mode = "basic",
                    bool go_to = true, bool automatic_release = false,
                    bool individual_control_fingers = false,
                    bool individual_control_scissor = false, int position = 0,
                    int speed = 0, int force = 0) override;
  void read_status() override;

 private:
  /// Modbus TCP context.
  modbus_t* mb;
};

// =========================================
// ModbusControllerSimulation implementation
// =========================================

/**
 * @brief Simulation controller that publishes joint commands directly.
 */
class ModbusControllerSimulation : public IModbusController {
 public:
  /**
   * @brief Construct simulation controller.
   * @param publisher Publisher for position controller commands.
   */
  ModbusControllerSimulation(
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher);

  void send_command(bool activate = true, const std::string& mode = "basic",
                    bool go_to = true, bool automatic_release = false,
                    bool individual_control_fingers = false,
                    bool individual_control_scissor = false, int position = 0,
                    int speed = 0, int force = 0) override;
  void read_status() override;

 private:
  /// Publisher used to publish joint commands on the controller topic.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

#endif  // ALLIANDER_ROBOTIQ__MODBUS_CONTROLLER_HPP_
