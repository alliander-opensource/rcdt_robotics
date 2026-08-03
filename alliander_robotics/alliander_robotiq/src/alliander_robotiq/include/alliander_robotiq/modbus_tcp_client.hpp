// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ALLIANDER_ROBOTIQ__MODBUS_TCP_CLIENT_HPP_
#define ALLIANDER_ROBOTIQ__MODBUS_TCP_CLIENT_HPP_

#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief Minimal Modbus TCP transport used by the hardware controller.
 */
class ModbusTcpClient {
 public:
  /**
   * @brief Construct transport client.
   * @param host IPv4 address of target device.
   * @param port Modbus TCP port.
   * @param timeout_seconds Socket send/receive timeout.
   */
  ModbusTcpClient(const std::string& host, int port, int timeout_seconds);

  /// Ensure socket resources are released.
  ~ModbusTcpClient();

  /**
   * @brief Open TCP socket and connect to target endpoint.
   * @return True on successful connection.
   */
  bool open();

  /**
   * @brief Write consecutive holding registers using function code 0x10.
   * @param start_address Starting register address.
   * @param registers Register values to write.
   * @return True when write succeeds and response is valid.
   */
  bool write_multiple_registers(std::uint16_t start_address,
                                const std::vector<std::uint16_t>& registers);

  /**
   * @brief Read consecutive input registers using function code 0x04.
   * @param start_address Starting register address.
   * @param quantity Number of registers to read.
   * @return Register vector, empty when request fails.
   */
  std::vector<std::uint16_t> read_input_registers(std::uint16_t start_address,
                                                  std::uint16_t quantity);

 private:
  /**
   * @brief Build MBAP (Modbus Application Protocol) frame and send a Modbus
   * PDU.
   * @param pdu Raw Modbus protocol data unit.
   * @return True when complete frame is transmitted.
   */
  bool send_request(const std::vector<std::uint8_t>& pdu);

  /**
   * @brief Receive MBAP response and extract Modbus PDU payload.
   * @param pdu Output buffer with received PDU.
   * @return True when a full response was received.
   */
  bool receive_response(std::vector<std::uint8_t>& pdu);

  /**
   * @brief Send all bytes of a buffer.
   * @param data Buffer pointer.
   * @param size Number of bytes to send.
   * @return True when all bytes were sent.
   */
  bool send_all(const std::uint8_t* data, std::size_t size);

  /**
   * @brief Receive exactly the requested number of bytes.
   * @param data Output buffer pointer.
   * @param size Number of bytes to receive.
   * @return True when all bytes were received.
   */
  bool recv_all(std::uint8_t* data, std::size_t size);

  /**
   * @brief Close the active socket if one is open.
   */
  void close_socket();

  /// Target host address.
  std::string host_;
  /// Target TCP port.
  int port_;
  /// Socket operation timeout in seconds.
  int timeout_seconds_;
  /// Active socket file descriptor, -1 when closed.
  int socket_fd_ = -1;
  /// Monotonic transaction identifier.
  std::uint16_t transaction_id_ = 0;
};

#endif  // ALLIANDER_ROBOTIQ__MODBUS_TCP_CLIENT_HPP_
