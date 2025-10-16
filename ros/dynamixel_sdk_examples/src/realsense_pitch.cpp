// Copyright 2025 Shimpei Sasaki
// This file manages the control of the Dynamixel motor and facilitates communication with ROS 2.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/float64.hpp"

#include "realsense_pitch.hpp"

// Control table addresses for AX-12A (Protocol 1.0) - These addresses are used to access motor parameters.
// Note: The AX-12A does not have an Operating Mode register.
#define ADDR_TORQUE_ENABLE 24    // AX-12A Torque Enable Address (1 Byte)
#define ADDR_GOAL_POSITION 30    // AX-12A Goal Position Address (2 Bytes)
#define ADDR_PRESENT_POSITION 36 // AX-12A Present Position Address (2 Bytes)

// Protocol version
#define PROTOCOL_VERSION 1.0     // AX-12A uses Protocol 1.0

// Default settings for motor communication.
#define BAUDRATE 1000000         // Common initial baudrate for AX-12A (change if necessary)
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;

constexpr int32_t MIN_GOAL_POSITION = 200;
constexpr int32_t MAX_GOAL_POSITION = 550;
constexpr uint8_t FIXED_DXL_ID = 5;

using namespace std::chrono_literals;

ReadWriteNode::ReadWriteNode()
: Node("realsense_pitch_node"),
  target_id_(FIXED_DXL_ID),
  has_recent_goal_(false),
  present_position_(0)
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  this->declare_parameter<int>("dynamixel_id", 1);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_position_subscriber_ =
    this->create_subscription<std_msgs::msg::Float64>(
    "set_position",
    QOS_RKL10V,
    [this](const std_msgs::msg::Float64::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

      const double requested_angle_deg = msg->data;
      // Add fixed offset (200) to map user angle to internal goal position
      // Goal = OFFSET + requested_angle_deg * 1023 / 300
      const double raw_goal = static_cast<double>(MIN_GOAL_POSITION) + requested_angle_deg * 1023.0 / 300.0;
      int32_t clamped_goal_position = static_cast<int32_t>(std::lround(raw_goal));
      if (clamped_goal_position < MIN_GOAL_POSITION) {
        clamped_goal_position = MIN_GOAL_POSITION;
      } else if (clamped_goal_position > MAX_GOAL_POSITION) {
        clamped_goal_position = MAX_GOAL_POSITION;
      }

      // The position value for AX-12A is represented as 2-byte data (uint16_t).
      uint16_t goal_position_uint16 = static_cast<uint16_t>(clamped_goal_position);

      // Write the Goal Position (2 bytes in length) to the motor.
      // Use write2ByteTxRx for AX-12A
      dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        target_id_,
        ADDR_GOAL_POSITION,
        goal_position_uint16,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        has_recent_goal_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "Set [ID: %u] [Goal Position: %d] (requested angle: %.2f deg)",
          static_cast<unsigned int>(target_id_),
          clamped_goal_position,
          requested_angle_deg
        );
      }
    }
    );

  present_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("present_angle", QOS_RKL10V);

  present_angle_timer_ = this->create_wall_timer(
    50ms,
    [this]() {
      if (!has_recent_goal_) {
        return;
      }

      uint16_t read_data_uint16 = 0;
      uint8_t local_error = 0;
      int local_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        target_id_,
        ADDR_PRESENT_POSITION,
        &read_data_uint16,
        &local_error
      );

      if (local_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(local_comm_result));
        return;
      }

      if (local_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(local_error));
        return;
      }

      present_position_ = static_cast<int32_t>(read_data_uint16);

      std_msgs::msg::Float64 angle_msg;
      // Convert the internal position to degrees using the same offset and scale as in set_position.
      // degree = (present_position - OFFSET) * 300 / 1023
      double angle = (static_cast<double>(present_position_) - static_cast<double>(MIN_GOAL_POSITION)) * 300.0 / 1023.0;
      // Ensure the angle is clamped within the range [0, 300].
      if (angle < 0.0) {
        angle = 0.0;
      } else if (angle > 300.0) {
        angle = 300.0;
      }
      angle_msg.data = angle;
      present_angle_publisher_->publish(angle_msg);
    });

  (void)0; // removed get_position service
}

ReadWriteNode::~ReadWriteNode()
{
}

void setupDynamixel(uint8_t dxl_id)
{
  // No need to set the Operating Mode for AX-12A (it is always in joint/wheel mode).

  // Enable the torque of the DYNAMIXEL motor.
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("realsense_pitch_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("realsense_pitch_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open the serial port for communication.
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("realsense_pitch_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("realsense_pitch_node"), "Succeeded to open the port.");
  }

  // Set the baud rate of the serial port (use the DYNAMIXEL baud rate).
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("realsense_pitch_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("realsense_pitch_node"), "Succeeded to set the baudrate.");
  }

// Enable torque on startup (use BROADCAST_ID=254 for all motors).
// It may be necessary to specify a specific ID like ID=1 instead of BROADCAST_ID depending on the environment.

  setupDynamixel(FIXED_DXL_ID); 

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL on shutdown
  packetHandler->write1ByteTxRx(
    portHandler,
    FIXED_DXL_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}