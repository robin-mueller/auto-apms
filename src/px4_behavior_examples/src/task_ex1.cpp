// Copyright 2024 Robin Müller
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

#include "px4_behavior/px4_behavior.hpp"
#include "px4_behavior_interfaces/action/arm_disarm.hpp"
#include "px4_behavior_interfaces/action/land.hpp"
#include "px4_behavior_interfaces/action/takeoff.hpp"

using namespace std::chrono_literals;
using ArmDisarmAction = px4_behavior_interfaces::action::ArmDisarm;
using TakeoffAction = px4_behavior_interfaces::action::Takeoff;
using LandAction = px4_behavior_interfaces::action::Land;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Create vehicle_command_client
    auto node_ptr = std::make_shared<rclcpp::Node>(std::string(EXAMPLE_NAME) + "_node");

    RCLCPP_INFO(node_ptr->get_logger(), "Running example '%s' ...", EXAMPLE_NAME);

    auto arm_disarm_client =
        px4_behavior::ActionClientWrapper<ArmDisarmAction>(*node_ptr, px4_behavior::ARM_DISARM_TASK_NAME);
    auto takeoff_client = px4_behavior::ActionClientWrapper<TakeoffAction>(*node_ptr, px4_behavior::TAKEOFF_TASK_NAME);
    auto land_client = px4_behavior::ActionClientWrapper<LandAction>(*node_ptr, px4_behavior::LAND_TASK_NAME);

    RCLCPP_INFO(node_ptr->get_logger(), "Arming ...");

    ArmDisarmAction::Goal arm_goal;
    arm_goal.arming_state = arm_goal.ARMING_STATE_ARM;
    auto arm_future = arm_disarm_client.SyncSendGoal(arm_goal);

    if (arm_future.wait_for(0s) == std::future_status::ready && !arm_future.get()) {
        std::cerr << "Arming goal was rejected\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    while (rclcpp::spin_until_future_complete(node_ptr, arm_future, 1s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_ptr->get_logger(), "Waiting for the vehicle to be armed");
        if (!rclcpp::ok()) {
            rclcpp::shutdown();
            return EXIT_FAILURE;
        }
    }

    auto arm_result = arm_future.get();

    if (!arm_result) {
        std::cerr << "Arm result is nullptr\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    if ((*arm_result).code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_ptr->get_logger(), "Arming succeeded");
    }
    else {
        std::cerr << "Arming did not succeed\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    RCLCPP_INFO(node_ptr->get_logger(), "Taking off ...");

    TakeoffAction::Goal takeoff_goal;
    takeoff_goal.altitude_amsl_m = 1e9;
    auto takeoff_future = takeoff_client.SyncSendGoal(takeoff_goal);

    if (takeoff_future.wait_for(0s) == std::future_status::ready && !takeoff_future.get()) {
        std::cerr << "Takeoff goal was rejected\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    // Wait for a few seconds here before aborting the takeoff task
    rclcpp::sleep_for(5s);

    RCLCPP_INFO(node_ptr->get_logger(), "Canceling takeoff ...");

    if (!takeoff_client.SyncCancelLastGoal()) {
        std::cerr << "Cancelation of takeoff was rejected\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    RCLCPP_INFO(node_ptr->get_logger(), "Landing ...");

    auto land_future = land_client.SyncSendGoal();

    if (land_future.wait_for(0s) == std::future_status::ready && !land_future.get()) {
        std::cerr << "Landing goal was rejected\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    while (rclcpp::spin_until_future_complete(node_ptr, land_future, 1s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_ptr->get_logger(), "Waiting for landing");
        if (!rclcpp::ok()) {
            rclcpp::shutdown();
            return EXIT_FAILURE;
        }
    }

    auto land_result = land_future.get();

    if (!land_result) {
        std::cerr << "Land result is nullptr\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    if ((*land_result).code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_ptr->get_logger(), "Landing succeeded");
    }
    else {
        std::cerr << "Landing did not succeed\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
