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

#include "auto_apms_interfaces/action/arm_disarm.hpp"

#include "auto_apms_px4/constants.hpp"
#include "auto_apms_px4/vehicle_command_client.hpp"
#include "auto_apms_util/action_wrapper.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

namespace auto_apms_px4
{

class ArmDisarmTask : public auto_apms_util::ActionWrapper<auto_apms_interfaces::action::ArmDisarm>
{
  enum State
  {
    WAIT_FOR_READY_TO_ARM,
    SEND_COMMAND,
    WAIT_FOR_ARMING_STATE_REACHED
  };

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_ptr_;
  State state_;
  bool ready_to_arm_{false};
  bool is_armed_{false};
  std::function<bool()> is_arming_state_reached_check__;
  std::function<bool()> send_command_callback_;
  const VehicleCommandClient vehicle_command_client_;

public:
  explicit ArmDisarmTask(const rclcpp::NodeOptions & options)
  : ActionWrapper{ARM_DISARM_TASK_NAME, options}, vehicle_command_client_{*this->node_ptr_}
  {
    vehicle_status_sub_ptr_ = this->node_ptr_->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", rclcpp::QoS(1).best_effort(), [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) {
        is_armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        ready_to_arm_ = msg->pre_flight_checks_pass;
      });
  }

private:
  bool onGoalRequest(std::shared_ptr<const Goal> goal_ptr) override final
  {
    // Reject goal if goal wants to arm, but UAV is not ready to arm and it is not requested to wait for ready to
    // arm
    if (goal_ptr->arming_state == Goal::ARMING_STATE_ARM && !goal_ptr->wait_until_ready_to_arm && !ready_to_arm_) {
      RCLCPP_ERROR(
        this->node_ptr_->get_logger(), "UAV can currently not be armed and wait_until_ready_to_arm is false.");
      return false;
    }

    if (goal_ptr->arming_state == Goal::ARMING_STATE_ARM) {
      // If goal is to arm
      state_ = ready_to_arm_ ? SEND_COMMAND : WAIT_FOR_READY_TO_ARM;
      is_arming_state_reached_check__ = [this]() { return is_armed_; };
      send_command_callback_ = [this]() { return vehicle_command_client_.Arm(); };
    } else {
      // If goal is to disarm
      state_ = SEND_COMMAND;
      is_arming_state_reached_check__ = [this]() { return !is_armed_; };
      send_command_callback_ = [this]() { return vehicle_command_client_.Disarm(); };
    }

    // Succeed directly if arming state is already reached
    if (
      (goal_ptr->arming_state == Goal::ARMING_STATE_ARM && is_armed_) ||
      (goal_ptr->arming_state == Goal::ARMING_STATE_DISARM && !is_armed_)) {
      state_ = WAIT_FOR_ARMING_STATE_REACHED;
    }

    return true;
  }

  Status executeGoal(
    std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> feedback_ptr,
    std::shared_ptr<Result> result_ptr) override final
  {
    (void)goal_ptr;
    (void)feedback_ptr;

    switch (state_) {
      case WAIT_FOR_READY_TO_ARM:
        if (ready_to_arm_) {
          state_ = SEND_COMMAND;
        }
        break;
      case SEND_COMMAND:
        if (send_command_callback_()) {
          state_ = WAIT_FOR_ARMING_STATE_REACHED;
        } else {
          RCLCPP_ERROR(this->node_ptr_->get_logger(), "Couldn't send arm/disarm command. Aborting...");
          result_ptr->state_changed = false;
          return Status::FAILURE;
        }
        break;
      case WAIT_FOR_ARMING_STATE_REACHED:
        if (is_arming_state_reached_check__()) {
          result_ptr->state_changed = true;
          return Status::SUCCESS;
        }
        break;
    }

    return Status::RUNNING;
  }
};

}  // namespace auto_apms_px4

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_px4::ArmDisarmTask)
