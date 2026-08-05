// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ALLIANDER_ROBOTIQ__MODBUS_CONTROLLER_HPP_
#define ALLIANDER_ROBOTIQ__MODBUS_CONTROLLER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

#include "alliander_robotiq/gripper_status.hpp"

/**
 * @brief Interface for hardware and simulation Modbus controllers.
 */
class IModbusController {
 public:
  /// Virtual destructor for interface polymorphism.
  virtual ~IModbusController() = default;

  /**
   * @brief Open controller communication channel when needed.
   * @return True if communication is ready.
   */
  virtual bool open() = 0;

  /**
   * @brief Send a command frame to the controller.
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

  /**
   * @brief Access mutable current controller status.
   * @return Reference to status object.
   */
  virtual GripperStatus& status() = 0;
};

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

  /**
   * @brief Destroy controller implementation.
   */
  ~ModbusControllerHardware() override;

  bool open() override;
  void send_command(bool activate = true, const std::string& mode = "basic",
                    bool go_to = true, bool automatic_release = false,
                    bool individual_control_fingers = false,
                    bool individual_control_scissor = false, int position = 0,
                    int speed = 0, int force = 0) override;
  void read_status() override;
  GripperStatus& status() override;

 private:
  /// Forward-declared implementation with socket transport details.
  class Impl;
  /// PIMPL pointer for ABI-safe transport implementation.
  std::unique_ptr<Impl> impl_;
};

/**
 * @brief Simulation controller that publishes joint commands directly.
 */
class ModbusControllerSimulation : public IModbusController {
 public:
  /**
   * @brief Construct simulation controller.
   * @param publisher Publisher for position controller commands.
   */
  explicit ModbusControllerSimulation(
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher);

  bool open() override;
  void send_command(bool activate = true, const std::string& mode = "basic",
                    bool go_to = true, bool automatic_release = false,
                    bool individual_control_fingers = false,
                    bool individual_control_scissor = false, int position = 0,
                    int speed = 0, int force = 0) override;
  void read_status() override;
  GripperStatus& status() override;

 private:
  /// Publisher used to command simulated joints.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  /// Internal status snapshot mirrored from the latest command.
  GripperStatus gripper_status_;
};

#endif  // ALLIANDER_ROBOTIQ__MODBUS_CONTROLLER_HPP_
