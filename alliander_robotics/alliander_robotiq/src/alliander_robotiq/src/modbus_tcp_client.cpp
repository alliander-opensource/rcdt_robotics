// SPDX-FileCopyrightText: Alliander N. V.
//
// SPDX-License-Identifier: Apache-2.0

#include "alliander_robotiq/modbus_tcp_client.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <utility>

ModbusTcpClient::ModbusTcpClient(const std::string& host, int port,
                                 int timeout_seconds)
    : host_(host), port_(port), timeout_seconds_(timeout_seconds) {}

ModbusTcpClient::~ModbusTcpClient() { close_socket(); }

void ModbusTcpClient::close_socket() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool ModbusTcpClient::open() {
  // Close any existing socket before opening a new one.
  close_socket();

  // Create a TCP socket.
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    return false;
  }

  // Set socket timeout for send/receive operations.
  struct timeval timeout;
  timeout.tv_sec = timeout_seconds_;
  setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(socket_fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  // Define the target address
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<std::uint16_t>(port_));
  if (inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1) {
    close_socket();
    return false;
  }

  // Connect to the target address
  if (connect(socket_fd_, reinterpret_cast<sockaddr*>(&address),
              sizeof(address)) != 0) {
    close_socket();
    return false;
  }

  return true;
}

bool ModbusTcpClient::write_multiple_registers(
    std::uint16_t start_address, const std::vector<std::uint16_t>& registers) {
  if (registers.empty()) {
    return false;
  }

  // There are 2 bytes per register
  const std::uint16_t quantity = static_cast<std::uint16_t>(registers.size());
  const std::uint8_t byte_count = static_cast<std::uint8_t>(quantity * 2U);

  // Create the PDU (Protocol Data Unit) and fill the header:
  // The header contains 6 bytes containing the function code (1 byte),
  // starting address (splitted in 2 parts, so 2 bytes), quantity of registers
  // (splitted in 2 parts, so 2 bytes), and byte count (1 byte).
  std::vector<std::uint8_t> pdu;
  pdu.reserve(6 + byte_count);
  pdu.push_back(0x10U);  // Function code for writing multiple registers
  pdu.push_back(
      static_cast<std::uint8_t>((start_address >> 8U) & 0xFFU));       // P1
  pdu.push_back(static_cast<std::uint8_t>(start_address & 0xFFU));     // P2
  pdu.push_back(static_cast<std::uint8_t>((quantity >> 8U) & 0xFFU));  // P1
  pdu.push_back(static_cast<std::uint8_t>(quantity & 0xFFU));          // P2
  pdu.push_back(byte_count);  // Number of bytes that will follow

  // Append register values to the PDU in big-endian order
  for (const auto value : registers) {
    pdu.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
    pdu.push_back(static_cast<std::uint8_t>(value & 0xFFU));
  }

  // Send the request and receive the response
  if (!send_request(pdu)) {
    return false;
  }
  std::vector<std::uint8_t> response;
  if (!receive_response(response)) {
    return false;
  }
  if (response.size() < 2 || response[0] == 0x90U) {
    return false;
  }
  return response[0] == 0x10U;
}

std::vector<std::uint16_t> ModbusTcpClient::read_input_registers(
    std::uint16_t start_address, std::uint16_t quantity) {
  // Create the PDU (Protocol Data Unit) and fill the header:
  // The header contains 5 bytes containing the function code (1 byte),
  // starting address (splitted in 2 parts, so 2 bytes) and quantity of
  // registers (splitted in 2 parts, so 2 bytes)
  std::vector<std::uint8_t> pdu;
  pdu.reserve(5);
  pdu.push_back(0x04U);  // Function code for reading input registers
  pdu.push_back(static_cast<std::uint8_t>((start_address >> 8U) & 0xFFU));
  pdu.push_back(static_cast<std::uint8_t>(start_address & 0xFFU));
  pdu.push_back(static_cast<std::uint8_t>((quantity >> 8U) & 0xFFU));
  pdu.push_back(static_cast<std::uint8_t>(quantity & 0xFFU));

  // Send the request and receive the response
  std::vector<std::uint16_t> output;
  if (!send_request(pdu)) {
    return output;
  }
  std::vector<std::uint8_t> response;
  if (!receive_response(response)) {
    return output;
  }
  if (response.size() < 2 || response[0] == 0x84U) {
    return output;
  }

  // Check that the response contains the expected number of bytes. A response
  // should contain 1 byte for the function code, 1 byte for the byte count,
  // and the 2 bytes for each register requested.
  const std::uint8_t byte_count = response[1];
  if (response.size() != static_cast<std::size_t>(2 + byte_count)) {
    return output;
  }

  // Extract the register values from the response. Each register is
  // represented by 2 bytes in big-endian order.
  output.reserve(byte_count / 2U);
  for (std::size_t idx = 0; idx + 1 < byte_count; idx += 2) {
    const std::uint16_t reg = static_cast<std::uint16_t>(
        (response[2 + idx] << 8U) | response[2 + idx + 1]);
    output.push_back(reg);
  }

  return output;
}

bool ModbusTcpClient::send_request(const std::vector<std::uint8_t>& pdu) {
  if (socket_fd_ < 0) {
    return false;
  }

  const std::uint16_t transaction_id = ++transaction_id_;
  const std::uint16_t protocol_id = 0;
  const std::uint16_t length = static_cast<std::uint16_t>(pdu.size() + 1U);
  const std::uint8_t unit_id = 1;

  // Create the MBAP (Modbus Application Protocol) header and fill it:
  // The header contains 7 bytes: transaction ID (2 bytes), protocol ID (2
  // bytes), length (2 bytes), and unit ID (1 byte). The PDU follows the header.
  std::vector<std::uint8_t> frame;
  frame.reserve(7 + pdu.size());
  frame.push_back(static_cast<std::uint8_t>((transaction_id >> 8U) & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>(transaction_id & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>((protocol_id >> 8U) & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>(protocol_id & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>((length >> 8U) & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>(length & 0xFFU));
  frame.push_back(unit_id);
  frame.insert(frame.end(), pdu.begin(), pdu.end());

  return send_all(frame.data(), frame.size());
}

bool ModbusTcpClient::send_all(const std::uint8_t* data, std::size_t size) {
  std::size_t sent = 0;
  while (sent < size) {
    const ssize_t bytes_sent = send(socket_fd_, data + sent, size - sent, 0);
    if (bytes_sent <= 0) {
      return false;
    }
    sent += static_cast<std::size_t>(bytes_sent);
  }
  return true;
}

bool ModbusTcpClient::receive_response(std::vector<std::uint8_t>& pdu) {
  pdu.clear();
  std::array<std::uint8_t, 7> header{};
  if (!recv_all(header.data(), header.size())) {
    return false;
  }

  const std::uint16_t length =
      static_cast<std::uint16_t>((header[4] << 8U) | header[5]);
  if (length == 0) {
    return false;
  }

  const std::size_t payload_size = static_cast<std::size_t>(length - 1U);
  std::vector<std::uint8_t> payload(payload_size);
  if (!recv_all(payload.data(), payload.size())) {
    return false;
  }

  pdu = std::move(payload);
  return true;
}

bool ModbusTcpClient::recv_all(std::uint8_t* data, std::size_t size) {
  std::size_t received = 0;
  while (received < size) {
    const ssize_t bytes_received =
        recv(socket_fd_, data + received, size - received, 0);
    if (bytes_received <= 0) {
      return false;
    }
    received += static_cast<std::size_t>(bytes_received);
  }
  return true;
}
