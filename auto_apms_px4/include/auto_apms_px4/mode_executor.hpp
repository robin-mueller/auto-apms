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

#include "auto_apms_px4/mode.hpp"
#include "auto_apms_px4/vehicle_command_client.hpp"
#include "auto_apms_util/action_wrapper.hpp"
#include "px4_msgs/msg/mode_completed.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_ros2/components/wait_for_fmu.hpp"
#include "px4_ros2/utils/message_version.hpp"

/**
 * @defgroup auto_apms_px4 PX4 Bridge
 * @brief Methods for using [PX4 Autopilot](https://px4.io/) together with AutoAPMS.
 *
 * We allow controlling autonomous sytems running PX4 by incorporating the [PX4/ROS2 Control
 * Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html) and allow the user to define custom
 * modes that can be dynamically registered with the autopilot. These modes are written in ROS 2 and run externally
 * while communicating with PX4 using the internal [uORB messages](https://docs.px4.io/main/en/middleware/uorb.html).
 */

/**
 * @ingroup auto_apms_px4
 * @brief Implementation of PX4 mode peers offered by [px4_ros2_cpp](https://github.com/Auterion/px4-ros2-interface-lib)
 * enabling integration with AutoAPMS.
 */
namespace auto_apms_px4
{

/**
 * @ingroup auto_apms_px4
 * @brief Generic template class for executing a PX4 mode implementing the interface of a standard ROS 2 action.
 *
 * The modes to be executed must be registered with the PX4 autopilot server before any action goals are sent. By
 * default, only the standard PX4 modes may be executed, but the user may also implement custom modes using ModeBase.
 * Refer to ModeExecutorFactory if you want to set up a ROS 2 node for executing your custom modes.
 *
 * ## Usage
 *
 * To register a ROS 2 node component that is able to execute for example the [land
 * mode](https://docs.px4.io/main/en/flight_modes_mc/land.html) when requested, the corresponding executor is
 * implemented as follows:
 *
 * ```cpp
 * #include "auto_apms_interfaces/action/takeoff.hpp"
 * #include "auto_apms_px4/mode_executor.hpp"
 *
 * namespace my_namespace
 * {
 * class MyTakeoffModeExecutor : public auto_apms_px4::ModeExecutor<auto_apms_interfaces::action::Takeoff>
 * {
 * public:
 *   explicit MyTakeoffModeExecutor(const rclcpp::NodeOptions & options)
 *   : ModeExecutor("my_executor_name", options, FlightMode::Takeoff)
 *   {
 *   }
 *
 *   bool sendActivationCommand(const VehicleCommandClient & client,
 *                              std::shared_ptr<const Goal> goal_ptr) override final
 *   {
 *     return client.takeoff(goal_ptr->altitude_amsl_m, goal_ptr->heading_rad);
 *   }
 * }
 * }  // namespace my_namespace
 *
 * // Register the ROS 2 node component
 * #include "rclcpp_components/register_node_macro.hpp"
 * RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::MyTakeoffModeExecutor)
 * ```
 *
 * @note The package `%auto_apms_px4` comes with ROS 2 node components for the most common standard modes and they work
 * out of the box.
 *
 * @tparam ActionT Type of the ROS 2 action.
 */
template <class ActionT>
class ModeExecutor : public auto_apms_util::ActionWrapper<ActionT>
{
  enum class State : uint8_t
  {
    REQUEST_ACTIVATION,
    WAIT_FOR_ACTIVATION,
    WAIT_FOR_COMPLETION_SIGNAL,
    COMPLETE
  };

public:
  using VehicleCommandClient = auto_apms_px4::VehicleCommandClient;
  using FlightMode = VehicleCommandClient::FlightMode;
  using typename auto_apms_util::ActionWrapper<ActionT>::ActionContextType;
  using typename auto_apms_util::ActionWrapper<ActionT>::Goal;
  using typename auto_apms_util::ActionWrapper<ActionT>::Feedback;
  using typename auto_apms_util::ActionWrapper<ActionT>::Result;
  using ActionStatus = auto_apms_util::ActionStatus;

  explicit ModeExecutor(
    const std::string & action_name, rclcpp::Node::SharedPtr node_ptr,
    std::shared_ptr<ActionContextType> action_context_ptr, uint8_t mode_id, bool deactivate_before_completion = true);
  explicit ModeExecutor(
    const std::string & action_name, const rclcpp::NodeOptions & options, uint8_t mode_id,
    bool deactivate_before_completion = true);
  explicit ModeExecutor(
    const std::string & action_name, const rclcpp::NodeOptions & options, FlightMode flight_mode,
    bool deactivate_before_completion = true);

private:
  void setUp();
  auto_apms_util::ActionStatus asyncDeactivateFlightMode();
  bool onGoalRequest(std::shared_ptr<const Goal> goal_ptr) override final;
  bool onCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override final;
  auto_apms_util::ActionStatus cancelGoal(
    std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) override final;
  auto_apms_util::ActionStatus executeGoal(
    std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> feedback_ptr,
    std::shared_ptr<Result> result_ptr) override final;

protected:
  bool isCurrentNavState(uint8_t nav_state);
  virtual bool sendActivationCommand(const VehicleCommandClient & client, std::shared_ptr<const Goal> goal_ptr);
  virtual bool isCompleted(std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus & vehicle_status);
  virtual void setFeedback(std::shared_ptr<Feedback> feedback_ptr, const px4_msgs::msg::VehicleStatus & vehicle_status);

  uint8_t getModeID() const;

private:
  const VehicleCommandClient vehicle_command_client_;
  const uint8_t mode_id_;
  bool deactivate_before_completion_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_ptr_;
  rclcpp::Subscription<px4_msgs::msg::ModeCompleted>::SharedPtr mode_completed_sub_ptr_;
  px4_msgs::msg::VehicleStatus::SharedPtr last_vehicle_status_ptr_;
  bool mode_completed_{false};
  bool deactivation_command_sent_{false};
  State state_{State::REQUEST_ACTIVATION};
  rclcpp::Time activation_command_sent_time_;
  rclcpp::Duration activation_timeout_{0, 0};
};

/**
 * @ingroup auto_apms_px4
 * @brief Helper template class that creates a ModeExecutor for a custom PX4 mode implemented by inheriting from
 * ModeBase.
 * @tparam ActionT Type of the ROS 2 action. Must be the same as used by @p ModeT.
 * @tparam ModeT Custom PX4 mode class.
 */
template <class ActionT, class ModeT>
class ModeExecutorFactory
{
public:
  ModeExecutorFactory(
    const std::string & action_name, const rclcpp::NodeOptions & options,
    const std::string & topic_namespace_prefix = "", bool deactivate_before_completion = true);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

private:
  rclcpp::Node::SharedPtr node_ptr_;  // It's necessary to also store the node pointer here for successful destruction
  std::unique_ptr<ModeBase<ActionT>> mode_ptr_;
  std::shared_ptr<ModeExecutor<ActionT>> mode_executor_ptr_;
};

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <class ActionT>
ModeExecutor<ActionT>::ModeExecutor(
  const std::string & action_name, rclcpp::Node::SharedPtr node_ptr,
  std::shared_ptr<ActionContextType> action_context_ptr, uint8_t mode_id, bool deactivate_before_completion)
: auto_apms_util::ActionWrapper<ActionT>(action_name, node_ptr, action_context_ptr),
  vehicle_command_client_(*node_ptr),
  mode_id_(mode_id),
  deactivate_before_completion_(deactivate_before_completion)
{
  setUp();
}

template <class ActionT>
ModeExecutor<ActionT>::ModeExecutor(
  const std::string & action_name, const rclcpp::NodeOptions & options, uint8_t mode_id,
  bool deactivate_before_completion)
: auto_apms_util::ActionWrapper<ActionT>(action_name, options),
  vehicle_command_client_(*this->node_ptr_),
  mode_id_(mode_id),
  deactivate_before_completion_(deactivate_before_completion)
{
  setUp();
}

template <class ActionT>
ModeExecutor<ActionT>::ModeExecutor(
  const std::string & action_name, const rclcpp::NodeOptions & options, FlightMode flight_mode,
  bool deactivate_before_completion)
: ModeExecutor<ActionT>(action_name, options, static_cast<uint8_t>(flight_mode), deactivate_before_completion)
{
}

template <class ActionT>
void ModeExecutor<ActionT>::setUp()
{
  vehicle_status_sub_ptr_ = this->node_ptr_->template create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleStatus>(),
    rclcpp::QoS(1).best_effort(),
    [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) { last_vehicle_status_ptr_ = std::move(msg); });

  mode_completed_sub_ptr_ = this->node_ptr_->template create_subscription<px4_msgs::msg::ModeCompleted>(
    "/fmu/out/mode_completed" + px4_ros2::getMessageNameVersion<px4_msgs::msg::ModeCompleted>(),
    rclcpp::QoS(1).best_effort(), [this](px4_msgs::msg::ModeCompleted::UniquePtr msg) {
      if (msg->nav_state == mode_id_) {
        if (msg->result == px4_msgs::msg::ModeCompleted::RESULT_SUCCESS) {
          this->mode_completed_ = true;
        } else {
          RCLCPP_ERROR(this->node_ptr_->get_logger(), "Flight mode %i failed to execute. Aborting...", this->mode_id_);
          this->action_context_ptr_->abort();
        }
        return;
      }
    });
}

template <class ActionT>
auto_apms_util::ActionStatus ModeExecutor<ActionT>::asyncDeactivateFlightMode()
{
  // If currently waiting for flight mode activation and HOLD is active we need to wait for the nav state to change
  // before starting deactivation. Otherwise, we'll misinterpret the current nav state when in
  // WAIT_FOR_HOLDING_STATE_REACHED and return success immediately
  bool is_holding = isCurrentNavState(static_cast<uint8_t>(FlightMode::Hold));
  if (state_ == State::WAIT_FOR_ACTIVATION) {
    if (is_holding) {
      auto & clock = *this->node_ptr_->get_clock();
      RCLCPP_DEBUG_THROTTLE(
        this->node_ptr_->get_logger(), clock, 250, "Waiting for flight mode %i to become active before deactivating...",
        mode_id_);
      return ActionStatus::RUNNING;
    } else {
      state_ = State::COMPLETE;  // Change state to indicate that mode has been activated
    }
  }

  if (is_holding) {
    RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Deactivated flight mode successfully (HOLD is active)");
    return ActionStatus::SUCCESS;
  } else {
    // Only send command if not in HOLD already
    if (!deactivation_command_sent_) {
      if (!vehicle_command_client_.syncActivateFlightMode(FlightMode::Hold)) {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "Failed to send command to activate HOLD");
        return ActionStatus::FAILURE;
      }
      // Force to consider only new status messages after sending new command
      last_vehicle_status_ptr_ = nullptr;
      deactivation_command_sent_ = true;
    }
  }

  return ActionStatus::RUNNING;
}

template <class ActionT>
bool ModeExecutor<ActionT>::onGoalRequest(const std::shared_ptr<const Goal> /*goal_ptr*/)
{
  state_ = State::REQUEST_ACTIVATION;
  deactivation_command_sent_ = false;
  mode_completed_ = false;
  activation_timeout_ = rclcpp::Duration::from_seconds(fmin(this->param_listener_.get_params().loop_rate * 15, 1.5));
  return true;
}

template <class ActionT>
bool ModeExecutor<ActionT>::onCancelRequest(
  std::shared_ptr<const Goal> /*goal_ptr*/, std::shared_ptr<Result> /*result_ptr*/)
{
  if (deactivate_before_completion_) {
    // To deactivate current flight mode, enable HOLD mode.
    RCLCPP_DEBUG(
      this->node_ptr_->get_logger(), "Cancellation requested! Will deactivate before termination (Enter HOLD)...");
  } else {
    RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Cancellation requested!");
  }
  return true;
}

template <class ActionT>
auto_apms_util::ActionStatus ModeExecutor<ActionT>::cancelGoal(
  std::shared_ptr<const Goal> /*goal_ptr*/, std::shared_ptr<Result> /*result_ptr*/)
{
  if (deactivate_before_completion_) {
    return asyncDeactivateFlightMode();
  }
  return ActionStatus::SUCCESS;
}

template <class ActionT>
bool ModeExecutor<ActionT>::isCurrentNavState(uint8_t nav_state)
{
  if (last_vehicle_status_ptr_ && last_vehicle_status_ptr_->nav_state == nav_state) {
    return true;
  }
  return false;
}

template <class ActionT>
auto_apms_util::ActionStatus ModeExecutor<ActionT>::executeGoal(
  std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Feedback> feedback_ptr, std::shared_ptr<Result> /*result_ptr*/)
{
  switch (state_) {
    case State::REQUEST_ACTIVATION:
      if (!sendActivationCommand(vehicle_command_client_, goal_ptr)) {
        RCLCPP_ERROR(
          this->node_ptr_->get_logger(), "Failed to send activation command for flight mode %i. Aborting...", mode_id_);
        return ActionStatus::FAILURE;
      }
      // Force to consider only new status messages after sending new command
      last_vehicle_status_ptr_ = nullptr;
      state_ = State::WAIT_FOR_ACTIVATION;
      activation_command_sent_time_ = this->node_ptr_->now();
      RCLCPP_DEBUG(
        this->node_ptr_->get_logger(), "Activation command for flight mode %i was sent successfully", mode_id_);
      return ActionStatus::RUNNING;
    case State::WAIT_FOR_ACTIVATION:
      if (isCurrentNavState(mode_id_)) {
        RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Flight mode %i is active", mode_id_);
        state_ = State::WAIT_FOR_COMPLETION_SIGNAL;
      } else if (this->node_ptr_->now() - activation_command_sent_time_ > activation_timeout_) {
        RCLCPP_ERROR(this->node_ptr_->get_logger(), "Timeout activating flight mode %i. Aborting...", mode_id_);
        return ActionStatus::FAILURE;
      }
      return ActionStatus::RUNNING;
    case State::WAIT_FOR_COMPLETION_SIGNAL:
      // Populate feedback message
      setFeedback(feedback_ptr, *last_vehicle_status_ptr_);

      // Check if execution should be terminated
      if (isCompleted(goal_ptr, *last_vehicle_status_ptr_)) {
        state_ = State::COMPLETE;
        if (deactivate_before_completion_) {
          // To deactivate current flight mode, enable HOLD mode
          RCLCPP_DEBUG(
            this->node_ptr_->get_logger(),
            "Flight mode %i complete! Will deactivate before termination (Enter HOLD)...", mode_id_);
        } else {
          RCLCPP_DEBUG(
            this->node_ptr_->get_logger(),
            "Flight mode %i complete! Will leave current navigation state as is. User is "
            "responsible for initiating the next flight mode...",
            mode_id_);
        }

        // Don't return to complete in same iteration
        break;
      }
      // Check if nav state changed
      if (!isCurrentNavState(mode_id_)) {
        RCLCPP_WARN(this->node_ptr_->get_logger(), "Flight mode %i was deactivated externally. Aborting...", mode_id_);
        return ActionStatus::FAILURE;
      }
      return ActionStatus::RUNNING;
    case State::COMPLETE:
      break;
  }

  if (deactivate_before_completion_) {
    const auto deactivation_state = asyncDeactivateFlightMode();
    if (deactivation_state != ActionStatus::SUCCESS) {
      return deactivation_state;
    }
    // Don't return to complete in same iteration
  }

  RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Flight mode %i execution termination", mode_id_);
  return ActionStatus::SUCCESS;
}

template <class ActionT>
bool ModeExecutor<ActionT>::sendActivationCommand(
  const VehicleCommandClient & client, std::shared_ptr<const Goal> /*goal_ptr*/)
{
  return client.syncActivateFlightMode(mode_id_);
}

template <class ActionT>
bool ModeExecutor<ActionT>::isCompleted(
  std::shared_ptr<const Goal> /*goal_ptr*/, const px4_msgs::msg::VehicleStatus & /*vehicle_status*/)
{
  return mode_completed_;
}

template <class ActionT>
void ModeExecutor<ActionT>::setFeedback(
  std::shared_ptr<Feedback> /*feedback_ptr*/, const px4_msgs::msg::VehicleStatus & /*vehicle_status*/)
{
  return;
}

template <class ActionT>
uint8_t ModeExecutor<ActionT>::getModeID() const
{
  return mode_id_;
}

template <class ActionT, class ModeT>
ModeExecutorFactory<ActionT, ModeT>::ModeExecutorFactory(
  const std::string & action_name, const rclcpp::NodeOptions & options, const std::string & topic_namespace_prefix,
  bool deactivate_before_completion)
: node_ptr_(std::make_shared<rclcpp::Node>(action_name + "_node", options))
{
  static_assert(
    std::is_base_of<ModeBase<ActionT>, ModeT>::value,
    "Template argument ModeT must inherit auto_apms::ModeBase<ActionT> as public and with same type "
    "ActionT as auto_apms::ActionWrapper<ActionT>");

  const auto action_context_ptr = std::make_shared<auto_apms_util::ActionContext<ActionT>>(node_ptr_->get_logger());

  mode_ptr_ = std::make_unique<ModeT>(
    *node_ptr_, px4_ros2::ModeBase::Settings(action_name), topic_namespace_prefix, action_context_ptr);

  if (!px4_ros2::waitForFMU(*node_ptr_, std::chrono::seconds(3))) {
    throw std::runtime_error("No message from FMU");
  } else {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "FMU availability test successful.");
  }

  if (!mode_ptr_->doRegister()) {
    RCLCPP_FATAL(node_ptr_->get_logger(), "Registration of mode with action_name: '%s' failed.", action_name.c_str());
    throw std::runtime_error("Mode registration failed");
  }

  // AFTER (!) registration, the mode id can be queried to set up the executor
  mode_executor_ptr_ = std::make_shared<ModeExecutor<ActionT>>(
    action_name, node_ptr_, action_context_ptr, mode_ptr_->id(), deactivate_before_completion);
}

template <class ActionT, class ModeT>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr ModeExecutorFactory<ActionT, ModeT>::get_node_base_interface()
{
  return node_ptr_->get_node_base_interface();
}

}  // namespace auto_apms_px4
