// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <optional>

#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros2/components/mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_px4
{

class VehicleCommandClient
{
  using VehicleCommand = px4_msgs::msg::VehicleCommand;

public:
  enum class FlightMode : uint8_t
  {
    Takeoff = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF,
    Land = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND,
    Hold = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER,
    RTL = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL,
    Mission = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION,
  };

  enum SendCommandResult : uint8_t
  {
    ACCEPTED,
    REJECTED,
    TIMEOUT
  };

  explicit VehicleCommandClient(
    rclcpp::Node & node, const std::chrono::milliseconds & command_timeout = std::chrono::milliseconds(500));

  // Send a vehicle command synchronously
  SendCommandResult SyncSendVehicleCommand(
    uint32_t command, float param1 = NAN, float param2 = NAN, float param3 = NAN, float param4 = NAN,
    float param5 = NAN, float param6 = NAN, float param7 = NAN) const;

  // Send a vehicle command synchronously
  SendCommandResult SyncSendVehicleCommand(const VehicleCommand & cmd) const;

  bool Arm() const;
  bool Disarm() const;

  /**
   * @brief (Re)starts the uploaded mission plan.
   *
   * If you want to resume a mission, use SyncActivateFlightMode(FlightMode::Mission) instead.
   */
  bool StartMission() const;

  bool Takeoff(float altitude_amsl_m, float heading_rad = NAN) const;

  bool Land() const;

  bool SyncActivateFlightMode(uint8_t mode_id) const;
  bool SyncActivateFlightMode(const FlightMode & mode) const;
  bool SyncActivateFlightMode(const px4_ros2::ModeBase * const mode_ptr) const;

private:
  rclcpp::Node & node_;
  const rclcpp::Logger logger_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
  const std::chrono::milliseconds command_timeout_;
};

std::string toStr(VehicleCommandClient::SendCommandResult result);

}  // namespace auto_apms_px4
