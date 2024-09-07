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

#include "px4_behavior/vehicle_command_client.hpp"

#include <functional>

namespace px4_behavior {

std::string to_string(VehicleCommandClient::SendCommandResult result)
{
    switch (result) {
        case VehicleCommandClient::SendCommandResult::ACCEPTED:
            return "ACCEPTED";
        case VehicleCommandClient::SendCommandResult::REJECTED:
            return "REJECTED";
        case VehicleCommandClient::SendCommandResult::TIMEOUT:
            return "TIMEOUT";
        default:
            return "undefined";
    }
}

VehicleCommandClient::VehicleCommandClient(rclcpp::Node& node, const std::chrono::milliseconds& command_timeout)
    : node_{node}, logger_{node.get_logger().get_child("vehicle_command_client")}, command_timeout_{command_timeout}
{
    // Create vehicle command publisher and acknowledgement signal subscriber
    vehicle_command_pub_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    vehicle_command_ack_sub_ = node_.create_subscription<px4_msgs::msg::VehicleCommandAck>(
        "/fmu/out/vehicle_command_ack",
        rclcpp::QoS(1).best_effort(),
        [](px4_msgs::msg::VehicleCommandAck::UniquePtr msg) { (void)msg; });
}

VehicleCommandClient::SendCommandResult VehicleCommandClient::SyncSendVehicleCommand(uint32_t command,
                                                                                     float param1,
                                                                                     float param2,
                                                                                     float param3,
                                                                                     float param4,
                                                                                     float param5,
                                                                                     float param6,
                                                                                     float param7) const
{
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.command = command;
    cmd.param1 = param1;
    cmd.param2 = param2;
    cmd.param3 = param3;
    cmd.param4 = param4;
    cmd.param5 = param5;
    cmd.param6 = param6;
    cmd.param7 = param7;
    cmd.timestamp = node_.get_clock()->now().nanoseconds() / 1000;

    return SyncSendVehicleCommand(cmd);
}

VehicleCommandClient::SendCommandResult VehicleCommandClient::SyncSendVehicleCommand(
    const px4_msgs::msg::VehicleCommand& cmd) const
{
    SendCommandResult result = SendCommandResult::REJECTED;
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(vehicle_command_ack_sub_);

    bool got_reply = false;
    auto start_time = std::chrono::steady_clock::now();
    vehicle_command_pub_->publish(cmd);

    while (!got_reply) {
        auto now = std::chrono::steady_clock::now();

        if (now >= start_time + command_timeout_) { break; }

        auto wait_ret = wait_set.wait(command_timeout_ - (now - start_time));

        if (wait_ret.kind() == rclcpp::WaitResultKind::Ready) {
            px4_msgs::msg::VehicleCommandAck ack;
            rclcpp::MessageInfo info;

            if (vehicle_command_ack_sub_->take(ack, info)) {
                if (ack.command == cmd.command && ack.target_component == cmd.source_component) {
                    RCLCPP_DEBUG(logger_,
                                 "SyncSendVehicleCommand: Command %i - received ack result %i",
                                 cmd.command,
                                 ack.result);
                    if (ack.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
                        result = SendCommandResult::ACCEPTED;
                    }
                    got_reply = true;
                }
            }
            else {
                RCLCPP_DEBUG(logger_, "SyncSendVehicleCommand: Command %i - message not valid", cmd.command);
            }
        }
    }

    wait_set.remove_subscription(vehicle_command_ack_sub_);

    if (!got_reply) {
        result = SendCommandResult::TIMEOUT;
        RCLCPP_WARN(logger_, "SyncSendVehicleCommand: Command %i - timeout, no ack received", cmd.command);
    }

    RCLCPP_DEBUG(logger_, "SyncSendVehicleCommand: Command %i - returned %s", cmd.command, to_string(result).c_str());

    return result;
}

bool VehicleCommandClient::Arm() const
{
    return !SyncSendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1);
}

bool VehicleCommandClient::Disarm() const
{
    return !SyncSendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0);
}

bool VehicleCommandClient::StartMission() const
{
    return !SyncSendVehicleCommand(VehicleCommand::VEHICLE_CMD_MISSION_START, 0);
}

bool VehicleCommandClient::Takeoff(float altitude_amsl_m, float heading_rad) const
{
    RCLCPP_DEBUG(logger_, "Takeoff parameters: altitude_amsl_m=%f heading_rad=%f", altitude_amsl_m, heading_rad);
    return !SyncSendVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
                                   NAN,
                                   NAN,
                                   NAN,
                                   heading_rad,
                                   NAN,
                                   NAN,
                                   altitude_amsl_m);
}

bool VehicleCommandClient::Land() const { return !SyncSendVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND); }

bool VehicleCommandClient::SyncActivateFlightMode(uint8_t mode_id) const
{
    return !SyncSendVehicleCommand(VehicleCommand::VEHICLE_CMD_SET_NAV_STATE, mode_id);
}

bool VehicleCommandClient::SyncActivateFlightMode(const FlightMode& mode) const
{
    return SyncActivateFlightMode(static_cast<uint8_t>(mode));
}

bool VehicleCommandClient::SyncActivateFlightMode(const px4_ros2::ModeBase* const mode_ptr) const
{
    return SyncActivateFlightMode(mode_ptr->id());
}

}  // namespace px4_behavior
