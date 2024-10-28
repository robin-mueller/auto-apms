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

#include "auto_apms_core/task.hpp"
#include "auto_apms_px4/mode.hpp"
#include "auto_apms_px4/vehicle_command_client.hpp"
#include "px4_msgs/msg/mode_completed.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros2/components/wait_for_fmu.hpp"

namespace auto_apms_px4
{

template <class ActionT>
class ModeExecutor : public auto_apms_core::Task<ActionT>
{
  enum class State : uint8_t
  {
    REQUEST_ACTIVATION,
    WAIT_FOR_ACTIVATION,
    WAIT_FOR_COMPLETION_SIGNAL,
    COMPLETE
  };

  using auto_apms_core::Task<ActionT>::DEFAULT_VALUE_EXECUTION_INTERVAL;
  using auto_apms_core::Task<ActionT>::DEFAULT_VALUE_FEEDBACK_INTERVAL;

public:
  using FlightMode = VehicleCommandClient::FlightMode;
  using typename auto_apms_core::Task<ActionT>::ActionContextType;
  using typename auto_apms_core::Task<ActionT>::Goal;
  using typename auto_apms_core::Task<ActionT>::Feedback;
  using typename auto_apms_core::Task<ActionT>::Result;
  using TaskStatus = auto_apms_core::TaskStatus;

  explicit ModeExecutor(const std::string& name, rclcpp::Node::SharedPtr node_ptr,
                        std::shared_ptr<ActionContextType> action_context_ptr, uint8_t mode_id,
                        bool deactivate_before_completion = true,
                        std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                        std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);
  explicit ModeExecutor(const std::string& name, const rclcpp::NodeOptions& options, uint8_t mode_id,
                        bool deactivate_before_completion = true,
                        std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                        std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);
  explicit ModeExecutor(const std::string& name, const rclcpp::NodeOptions& options, FlightMode flight_mode,
                        bool deactivate_before_completion = true,
                        std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                        std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);

private:
  void SetUp();
  auto_apms_core::TaskStatus AsyncDeactivateFlightMode();
  bool OnGoalRequest(std::shared_ptr<const Goal> goal_ptr) final;
  bool OnCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) final;
  auto_apms_core::TaskStatus CancelGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) final;
  auto_apms_core::TaskStatus ExecuteGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> feedback_ptr,
                                         std::shared_ptr<Result> result_ptr) final;

protected:
  bool IsCurrentNavState(uint8_t nav_state);
  virtual bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr);
  virtual bool IsCompleted(std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus& vehicle_status);
  virtual void SetFeedback(std::shared_ptr<Feedback> feedback_ptr, const px4_msgs::msg::VehicleStatus& vehicle_status);

  uint8_t mode_id() const;

private:
  const VehicleCommandClient vehicle_command_client_;
  const uint8_t mode_id_;
  bool deactivate_before_completion_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_ptr_;
  rclcpp::Subscription<px4_msgs::msg::ModeCompleted>::SharedPtr mode_completed_sub_ptr_;
  px4_msgs::msg::VehicleStatus::SharedPtr last_vehicle_status_ptr_;
  bool mode_completed_{ false };
  bool deactivation_command_sent_{ false };
  State state_{ State::REQUEST_ACTIVATION };
};

template <class ActionT>
ModeExecutor<ActionT>::ModeExecutor(const std::string& name, rclcpp::Node::SharedPtr node_ptr,
                                    std::shared_ptr<ActionContextType> action_context_ptr, uint8_t mode_id,
                                    bool deactivate_before_completion, std::chrono::milliseconds execution_interval,
                                    std::chrono::milliseconds feedback_interval)
  : auto_apms_core::Task<ActionT>{ name, node_ptr, action_context_ptr, execution_interval, feedback_interval }
  , vehicle_command_client_{ *node_ptr }
  , mode_id_{ mode_id }
  , deactivate_before_completion_{ deactivate_before_completion }
{
  SetUp();
}

template <class ActionT>
ModeExecutor<ActionT>::ModeExecutor(const std::string& name, const rclcpp::NodeOptions& options, uint8_t mode_id,
                                    bool deactivate_before_completion, std::chrono::milliseconds execution_interval,
                                    std::chrono::milliseconds feedback_interval)
  : auto_apms_core::Task<ActionT>{ name, options, execution_interval, feedback_interval }
  , vehicle_command_client_{ *this->node_ptr_ }
  , mode_id_{ mode_id }
  , deactivate_before_completion_{ deactivate_before_completion }
{
  SetUp();
}

template <class ActionT>
ModeExecutor<ActionT>::ModeExecutor(const std::string& name, const rclcpp::NodeOptions& options, FlightMode flight_mode,
                                    bool deactivate_before_completion, std::chrono::milliseconds execution_interval,
                                    std::chrono::milliseconds feedback_interval)
  : ModeExecutor<ActionT>{ name,
                           options,
                           static_cast<uint8_t>(flight_mode),
                           deactivate_before_completion,
                           execution_interval,
                           feedback_interval }
{
}

template <class ActionT>
void ModeExecutor<ActionT>::SetUp()
{
  vehicle_status_sub_ptr_ = this->node_ptr_->template create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status", rclcpp::QoS(1).best_effort(),
      [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) { last_vehicle_status_ptr_ = std::move(msg); });

  mode_completed_sub_ptr_ = this->node_ptr_->template create_subscription<px4_msgs::msg::ModeCompleted>(
      "/fmu/out/mode_completed", rclcpp::QoS(1).best_effort(), [this](px4_msgs::msg::ModeCompleted::UniquePtr msg) {
        if (msg->nav_state == mode_id_)
        {
          if (msg->result == px4_msgs::msg::ModeCompleted::RESULT_SUCCESS)
          {
            this->mode_completed_ = true;
          }
          else
          {
            RCLCPP_ERROR(this->node_ptr_->get_logger(), "Flight mode %i failed to execute. Aborting...",
                         this->mode_id_);
            this->action_context_ptr_->Abort();
          }
          return;
        }
      });
}

template <class ActionT>
auto_apms_core::TaskStatus ModeExecutor<ActionT>::AsyncDeactivateFlightMode()
{
  // If currently waiting for flight mode activation and HOLD is active we need to wait for the nav state to change
  // before starting deactivation. Otherwise, we'll misinterpret the current nav state when in
  // WAIT_FOR_HOLDING_STATE_REACHED and return success immediately
  bool is_holding = IsCurrentNavState(static_cast<uint8_t>(FlightMode::Hold));
  if (state_ == State::WAIT_FOR_ACTIVATION)
  {
    if (is_holding)
    {
      auto& clock = *this->node_ptr_->get_clock();
      RCLCPP_DEBUG_THROTTLE(this->node_ptr_->get_logger(), clock, 200,
                            "Waiting for flight mode %i to become active before deactivating...", mode_id_);
      return TaskStatus::RUNNING;
    }
    else
    {
      state_ = State::COMPLETE;  // Change state to indicate that mode has been activated
    }
  }

  if (is_holding)
  {
    RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Deactivated flight mode successfully (HOLD is active)");
    return TaskStatus::SUCCESS;
  }
  else
  {
    // Only send command if not in HOLD already
    if (!deactivation_command_sent_)
    {
      if (!vehicle_command_client_.SyncActivateFlightMode(FlightMode::Hold))
      {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "Failed to send command to activate HOLD");
        return TaskStatus::FAILURE;
      }
      // Force to consider only new status messages after sending new command
      last_vehicle_status_ptr_ = nullptr;
      deactivation_command_sent_ = true;
    }
  }

  return TaskStatus::RUNNING;
}

template <class ActionT>
bool ModeExecutor<ActionT>::OnGoalRequest(const std::shared_ptr<const Goal> goal_ptr)
{
  (void)goal_ptr;
  state_ = State::REQUEST_ACTIVATION;
  deactivation_command_sent_ = false;
  mode_completed_ = false;
  return true;
}

template <class ActionT>
bool ModeExecutor<ActionT>::OnCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr)
{
  (void)goal_ptr;
  (void)result_ptr;
  if (deactivate_before_completion_)
  {
    // To deactivate current flight mode, enable HOLD mode.
    RCLCPP_DEBUG(this->node_ptr_->get_logger(),
                 "Cancelation requested! Will deactivate before termination (Enter HOLD)...");
  }
  else
  {
    RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Cancelation requested!");
  }
  return true;
}

template <class ActionT>
auto_apms_core::TaskStatus ModeExecutor<ActionT>::CancelGoal(std::shared_ptr<const Goal> goal_ptr,
                                                             std::shared_ptr<Result> result_ptr)
{
  (void)goal_ptr;
  (void)result_ptr;
  if (deactivate_before_completion_)
  {
    return AsyncDeactivateFlightMode();
  }
  return TaskStatus::SUCCESS;
}

template <class ActionT>
bool ModeExecutor<ActionT>::IsCurrentNavState(uint8_t nav_state)
{
  if (last_vehicle_status_ptr_ && last_vehicle_status_ptr_->nav_state == nav_state)
  {
    return true;
  }
  return false;
}

template <class ActionT>
auto_apms_core::TaskStatus ModeExecutor<ActionT>::ExecuteGoal(std::shared_ptr<const Goal> goal_ptr,
                                                              std::shared_ptr<Feedback> feedback_ptr,
                                                              std::shared_ptr<Result> result_ptr)
{
  (void)goal_ptr;
  (void)result_ptr;

  switch (state_)
  {
    case State::REQUEST_ACTIVATION:
      if (!SendActivationCommand(vehicle_command_client_, goal_ptr))
      {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "Failed to send activation command for flight mode %i. Aborting...",
                     mode_id_);
        return TaskStatus::FAILURE;
      }
      // Force to consider only new status messages after sending new command
      last_vehicle_status_ptr_ = nullptr;
      state_ = State::WAIT_FOR_ACTIVATION;

      RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Activation command for flight mode %i was sent successfully",
                   mode_id_);
      return TaskStatus::RUNNING;
    case State::WAIT_FOR_ACTIVATION:
      if (IsCurrentNavState(mode_id_))
      {
        RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Flight mode %i is active", mode_id_);
        state_ = State::WAIT_FOR_COMPLETION_SIGNAL;
      }
      return TaskStatus::RUNNING;
    case State::WAIT_FOR_COMPLETION_SIGNAL:
      // Populate feedback message
      SetFeedback(feedback_ptr, *last_vehicle_status_ptr_);

      // Check if execution should be terminated
      if (IsCompleted(goal_ptr, *last_vehicle_status_ptr_))
      {
        state_ = State::COMPLETE;
        if (deactivate_before_completion_)
        {
          // To deactivate current flight mode, enable HOLD mode
          RCLCPP_DEBUG(this->node_ptr_->get_logger(),
                       "Flight mode %i complete! Will deactivate before termination (Enter HOLD)...", mode_id_);
        }
        else
        {
          RCLCPP_DEBUG(this->node_ptr_->get_logger(),
                       "Flight mode %i complete! Will leave current navigation state as is. User is "
                       "responsible for initiating the next flight mode...",
                       mode_id_);
        }

        // Don't return to complete in same iteration
        break;
      }
      // Check if nav state changed
      if (!IsCurrentNavState(mode_id_))
      {
        RCLCPP_WARN(this->node_ptr_->get_logger(), "Flight mode %i was deactivated externally. Aborting...", mode_id_);
        return TaskStatus::FAILURE;
      }
      return TaskStatus::RUNNING;
    case State::COMPLETE:
      break;
  }

  if (deactivate_before_completion_)
  {
    const auto deactivation_state = AsyncDeactivateFlightMode();
    if (deactivation_state != TaskStatus::SUCCESS)
    {
      return deactivation_state;
    }
    // Don't return to complete in same iteration
  }

  RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Flight mode %i execution termination", mode_id_);
  return TaskStatus::SUCCESS;
}

template <class ActionT>
bool ModeExecutor<ActionT>::SendActivationCommand(const VehicleCommandClient& client,
                                                  std::shared_ptr<const Goal> goal_ptr)
{
  (void)goal_ptr;
  return client.SyncActivateFlightMode(mode_id_);
}

template <class ActionT>
bool ModeExecutor<ActionT>::IsCompleted(std::shared_ptr<const Goal> goal_ptr,
                                        const px4_msgs::msg::VehicleStatus& vehicle_status)
{
  (void)goal_ptr;
  (void)vehicle_status;
  return mode_completed_;
}

template <class ActionT>
void ModeExecutor<ActionT>::SetFeedback(std::shared_ptr<Feedback> feedback_ptr,
                                        const px4_msgs::msg::VehicleStatus& vehicle_status)
{
  (void)feedback_ptr;
  (void)vehicle_status;
  return;
}

template <class ActionT>
uint8_t ModeExecutor<ActionT>::mode_id() const
{
  return mode_id_;
}

template <class ActionT, class ModeT>
class ModeExecutorFactory
{
public:
  ModeExecutorFactory(
      const std::string& name, const rclcpp::NodeOptions& options, const std::string& topic_namespace_prefix = "",
      bool deactivate_before_completion = true,
      std::chrono::milliseconds execution_interval = auto_apms_core::Task<ActionT>::DEFAULT_VALUE_EXECUTION_INTERVAL,
      std::chrono::milliseconds feedback_interval = auto_apms_core::Task<ActionT>::DEFAULT_VALUE_FEEDBACK_INTERVAL);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

private:
  rclcpp::Node::SharedPtr node_ptr_;  // It's necessary to also store the node pointer here for successful destruction
  std::unique_ptr<ModeBase<ActionT>> mode_ptr_;
  std::shared_ptr<ModeExecutor<ActionT>> mode_executor_ptr_;
};

template <class ActionT, class ModeT>
ModeExecutorFactory<ActionT, ModeT>::ModeExecutorFactory(const std::string& name, const rclcpp::NodeOptions& options,
                                                         const std::string& topic_namespace_prefix,
                                                         bool deactivate_before_completion,
                                                         std::chrono::milliseconds execution_interval,
                                                         std::chrono::milliseconds feedback_interval)
  : node_ptr_{ std::make_shared<rclcpp::Node>("task_" + name + "_node", options) }
{
  static_assert(std::is_base_of<ModeBase<ActionT>, ModeT>::value,
                "Template argument ModeT must inherit auto_apms::ModeBase<ActionT> as public and with same type "
                "ActionT as auto_apms::Task<ActionT>");

  const auto action_context_ptr = std::make_shared<auto_apms_core::ActionContext<ActionT>>(node_ptr_->get_logger());

  mode_ptr_ = std::make_unique<ModeT>(*node_ptr_, px4_ros2::ModeBase::Settings{ "mode_" + name },
                                      topic_namespace_prefix, action_context_ptr);

  if (!px4_ros2::waitForFMU(*node_ptr_, std::chrono::seconds(3)))
  {
    throw std::runtime_error("No message from FMU");
  }
  else
  {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "FMU availability test successful.");
  }

  if (!mode_ptr_->doRegister())
  {
    RCLCPP_FATAL(node_ptr_->get_logger(), "Registration of mode with task_name: '%s' failed.", name.c_str());
    throw std::runtime_error("Mode registration failed");
  }

  // AFTER (!) registration, the mode id can be queried to set up the executor
  mode_executor_ptr_ =
      std::make_shared<ModeExecutor<ActionT>>(name, node_ptr_, action_context_ptr, mode_ptr_->id(),
                                              deactivate_before_completion, execution_interval, feedback_interval);
}

template <class ActionT, class ModeT>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ModeExecutorFactory<ActionT, ModeT>::get_node_base_interface()
{
  return node_ptr_->get_node_base_interface();
}

}  // namespace auto_apms_px4
