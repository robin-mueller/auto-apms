#pragma once

#include <px4_behavior/maneuver/maneuver.hpp>
#include <px4_behavior/maneuver/maneuver_mode.hpp>
#include <px4_behavior/vehicle_command_client.hpp>
#include <px4_msgs/msg/mode_completed.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>

namespace px4_behavior {

template <class ActionT>
class FlightModeExecutor : public Maneuver<ActionT>
{
    enum class State : uint8_t { REQUEST_ACTIVATION, WAIT_FOR_ACTIVATION, WAIT_FOR_COMPLETION_SIGNAL, COMPLETE };

   public:
    using FlightMode = VehicleCommandClient::FlightMode;
    using Goal = typename ActionContext<ActionT>::Goal;
    using Feedback = typename ActionContext<ActionT>::Feedback;
    using Result = typename ActionContext<ActionT>::Result;

    explicit FlightModeExecutor(const std::string& maneuver_name,
                                rclcpp::Node::SharedPtr node_ptr,
                                std::shared_ptr<ActionContext<ActionT>> action_context_ptr,
                                uint8_t mode_id,
                                bool deactivate_before_completion = true,
                                std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                                std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);
    explicit FlightModeExecutor(const std::string& maneuver_name,
                                const rclcpp::NodeOptions& options,
                                uint8_t mode_id,
                                bool deactivate_before_completion = true,
                                std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                                std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);
    explicit FlightModeExecutor(const std::string& maneuver_name,
                                const rclcpp::NodeOptions& options,
                                FlightMode flight_mode,
                                bool deactivate_before_completion = true,
                                std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                                std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);

   private:
    void SetUp();
    ManeuverExecutionState AsyncDeactivateFlightMode();
    bool OnGoalRequest(std::shared_ptr<const Goal> goal_ptr) final;
    bool OnCancelRequest(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) final;
    ManeuverExecutionState CancelGoal(std::shared_ptr<const Goal> goal_ptr, std::shared_ptr<Result> result_ptr) final;
    ManeuverExecutionState ExecuteGoal(std::shared_ptr<const Goal> goal_ptr,
                                       std::shared_ptr<Feedback> feedback_ptr,
                                       std::shared_ptr<Result> result_ptr) final;

   protected:
    bool IsCurrentNavState(uint8_t nav_state);
    virtual bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr);
    virtual bool IsCompleted(std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus& vehicle_status);
    virtual void SetFeedback(std::shared_ptr<Feedback> feedback_ptr,
                             const px4_msgs::msg::VehicleStatus& vehicle_status);

    uint8_t mode_id() const;

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
};

template <class ActionT>
FlightModeExecutor<ActionT>::FlightModeExecutor(const std::string& maneuver_name,
                                                rclcpp::Node::SharedPtr node_ptr,
                                                std::shared_ptr<ActionContext<ActionT>> action_context_ptr,
                                                uint8_t mode_id,
                                                bool deactivate_before_completion,
                                                std::chrono::milliseconds execution_interval,
                                                std::chrono::milliseconds feedback_interval)
    : Maneuver<ActionT>{maneuver_name, node_ptr, action_context_ptr, execution_interval, feedback_interval},
      vehicle_command_client_{*node_ptr},
      mode_id_{mode_id},
      deactivate_before_completion_{deactivate_before_completion}
{
    SetUp();
}

template <class ActionT>
FlightModeExecutor<ActionT>::FlightModeExecutor(const std::string& maneuver_name,
                                                const rclcpp::NodeOptions& options,
                                                uint8_t mode_id,
                                                bool deactivate_before_completion,
                                                std::chrono::milliseconds execution_interval,
                                                std::chrono::milliseconds feedback_interval)
    : Maneuver<ActionT>{maneuver_name, options, execution_interval, feedback_interval},
      vehicle_command_client_{*this->node_ptr_},
      mode_id_{mode_id},
      deactivate_before_completion_{deactivate_before_completion}
{
    SetUp();
}

template <class ActionT>
FlightModeExecutor<ActionT>::FlightModeExecutor(const std::string& maneuver_name,
                                                const rclcpp::NodeOptions& options,
                                                FlightMode flight_mode,
                                                bool deactivate_before_completion,
                                                std::chrono::milliseconds execution_interval,
                                                std::chrono::milliseconds feedback_interval)
    : FlightModeExecutor<ActionT>{maneuver_name,
                                  options,
                                  static_cast<uint8_t>(flight_mode),
                                  deactivate_before_completion,
                                  execution_interval,
                                  feedback_interval}
{}

template <class ActionT>
void FlightModeExecutor<ActionT>::SetUp()
{
    vehicle_status_sub_ptr_ = this->node_ptr_->template create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status",
        rclcpp::QoS(1).best_effort(),
        [this](px4_msgs::msg::VehicleStatus::UniquePtr msg) { last_vehicle_status_ptr_ = std::move(msg); });

    mode_completed_sub_ptr_ = this->node_ptr_->template create_subscription<px4_msgs::msg::ModeCompleted>(
        "/fmu/out/mode_completed",
        rclcpp::QoS(1).best_effort(),
        [this](px4_msgs::msg::ModeCompleted::UniquePtr msg) {
            if (msg->nav_state == mode_id_) {
                if (msg->result == px4_msgs::msg::ModeCompleted::RESULT_SUCCESS) { this->mode_completed_ = true; }
                else {
                    RCLCPP_ERROR(this->node_ptr_->get_logger(),
                                 "Flight mode %i failed to execute. Aborting...",
                                 this->mode_id_);
                    this->action_context_ptr_->Abort();
                }
                return;
            }
        });
}

template <class ActionT>
ManeuverExecutionState FlightModeExecutor<ActionT>::AsyncDeactivateFlightMode()
{
    // If currently waiting for flight mode activation and HOLD is active we need to wait for the nav state to change
    // before starting deactivation. Otherwise, we'll misinterpret the current nav state when in
    // WAIT_FOR_HOLDING_STATE_REACHED and return success immediately
    bool is_holding = IsCurrentNavState(static_cast<uint8_t>(FlightMode::Hold));
    if (state_ == State::WAIT_FOR_ACTIVATION) {
        if (is_holding) {
            auto& clock = *this->node_ptr_->get_clock();
            RCLCPP_DEBUG_THROTTLE(this->node_ptr_->get_logger(),
                                  clock,
                                  200,
                                  "Waiting for flight mode %i to become active before deactivating...",
                                  mode_id_);
            return ManeuverExecutionState::RUNNING;
        }
        else {
            state_ = State::COMPLETE;  // Change state to indicate that mode has been activated
        }
    }

    if (is_holding) {
        RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Deactivated flight mode successfully (HOLD is active)");
        return ManeuverExecutionState::SUCCESS;
    }
    else {
        // Only send command if not in HOLD already
        if (!deactivation_command_sent_) {
            if (!vehicle_command_client_.SyncActivateFlightMode(FlightMode::Hold)) {
                RCLCPP_ERROR(this->node_ptr_->get_logger(), "Failed to send command to activate HOLD");
                return ManeuverExecutionState::FAILURE;
            }
            // Force to consider only new status messages after sending new command
            last_vehicle_status_ptr_ = nullptr;
            deactivation_command_sent_ = true;
        }
    }

    return ManeuverExecutionState::RUNNING;
}

template <class ActionT>
bool FlightModeExecutor<ActionT>::OnGoalRequest(const std::shared_ptr<const Goal> goal_ptr)
{
    (void)goal_ptr;
    state_ = State::REQUEST_ACTIVATION;
    deactivation_command_sent_ = false;
    mode_completed_ = false;
    return true;
}

template <class ActionT>
bool FlightModeExecutor<ActionT>::OnCancelRequest(std::shared_ptr<const Goal> goal_ptr,
                                                  std::shared_ptr<Result> result_ptr)
{
    (void)goal_ptr;
    (void)result_ptr;
    if (deactivate_before_completion_) {
        // To deactivate current flight mode, enable HOLD mode.
        RCLCPP_DEBUG(this->node_ptr_->get_logger(),
                     "Cancelation requested! Will deactivate before termination (Enter HOLD)...");
    }
    else {
        RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Cancelation requested!");
    }
    return true;
}

template <class ActionT>
ManeuverExecutionState FlightModeExecutor<ActionT>::CancelGoal(std::shared_ptr<const Goal> goal_ptr,
                                                               std::shared_ptr<Result> result_ptr)
{
    (void)goal_ptr;
    (void)result_ptr;
    if (deactivate_before_completion_) return AsyncDeactivateFlightMode();
    return ManeuverExecutionState::SUCCESS;
}

template <class ActionT>
bool FlightModeExecutor<ActionT>::IsCurrentNavState(uint8_t nav_state)
{
    if (last_vehicle_status_ptr_ && last_vehicle_status_ptr_->nav_state == nav_state) { return true; }
    return false;
}

template <class ActionT>
ManeuverExecutionState FlightModeExecutor<ActionT>::ExecuteGoal(std::shared_ptr<const Goal> goal_ptr,
                                                                std::shared_ptr<Feedback> feedback_ptr,
                                                                std::shared_ptr<Result> result_ptr)
{
    (void)goal_ptr;
    (void)result_ptr;

    switch (state_) {
        case State::REQUEST_ACTIVATION:
            if (!SendActivationCommand(vehicle_command_client_, goal_ptr)) {
                RCLCPP_ERROR(this->node_ptr_->get_logger(),
                             "Failed to send activation command for flight mode %i. Aborting...",
                             mode_id_);
                return ManeuverExecutionState::FAILURE;
            }
            // Force to consider only new status messages after sending new command
            last_vehicle_status_ptr_ = nullptr;
            state_ = State::WAIT_FOR_ACTIVATION;

            RCLCPP_DEBUG(this->node_ptr_->get_logger(),
                         "Activation command for flight mode %i was sent successfully",
                         mode_id_);
            return ManeuverExecutionState::RUNNING;
        case State::WAIT_FOR_ACTIVATION:
            if (IsCurrentNavState(mode_id_)) {
                RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Flight mode %i is active", mode_id_);
                state_ = State::WAIT_FOR_COMPLETION_SIGNAL;
            }
            return ManeuverExecutionState::RUNNING;
        case State::WAIT_FOR_COMPLETION_SIGNAL:
            // Populate feedback message
            SetFeedback(feedback_ptr, *last_vehicle_status_ptr_);

            // Check if execution should be terminated
            if (IsCompleted(goal_ptr, *last_vehicle_status_ptr_)) {
                state_ = State::COMPLETE;
                if (deactivate_before_completion_) {
                    // To deactivate current flight mode, enable HOLD mode
                    RCLCPP_DEBUG(this->node_ptr_->get_logger(),
                                 "Flight mode %i complete! Will deactivate before termination (Enter HOLD)...",
                                 mode_id_);
                }
                else {
                    RCLCPP_DEBUG(this->node_ptr_->get_logger(),
                                 "Flight mode %i complete! Will leave current navigation state as is. User is "
                                 "responsible for initiating the next flight mode...",
                                 mode_id_);
                }

                // Don't return to complete in same iteration
                break;
            }
            // Check if nav state changed
            if (!IsCurrentNavState(static_cast<uint8_t>(mode_id_))) {
                RCLCPP_WARN(this->node_ptr_->get_logger(),
                            "Flight mode %i was deactivated externally. Aborting...",
                            mode_id_);
                return ManeuverExecutionState::FAILURE;
            }
            return ManeuverExecutionState::RUNNING;
        case State::COMPLETE:
            break;
    }

    if (deactivate_before_completion_) {
        const auto deactivation_state = AsyncDeactivateFlightMode();
        if (deactivation_state != ManeuverExecutionState::SUCCESS) return deactivation_state;
        // Don't return to complete in same iteration
    }

    RCLCPP_DEBUG(this->node_ptr_->get_logger(), "Flight mode %i execution termination", mode_id_);
    return ManeuverExecutionState::SUCCESS;
}

template <class ActionT>
bool FlightModeExecutor<ActionT>::SendActivationCommand(const VehicleCommandClient& client,
                                                        std::shared_ptr<const Goal> goal_ptr)
{
    (void)goal_ptr;
    return client.SyncActivateFlightMode(mode_id_);
}

template <class ActionT>
bool FlightModeExecutor<ActionT>::IsCompleted(std::shared_ptr<const Goal> goal_ptr,
                                              const px4_msgs::msg::VehicleStatus& vehicle_status)
{
    (void)goal_ptr;
    (void)vehicle_status;
    return mode_completed_;
}

template <class ActionT>
void FlightModeExecutor<ActionT>::SetFeedback(std::shared_ptr<Feedback> feedback_ptr,
                                              const px4_msgs::msg::VehicleStatus& vehicle_status)
{
    (void)feedback_ptr;
    (void)vehicle_status;
    return;
}

template <class ActionT>
uint8_t FlightModeExecutor<ActionT>::mode_id() const
{
    return mode_id_;
}

template <class ActionT, class ModeT>
class ExternalFlightModeExecutor
{
   public:
    ExternalFlightModeExecutor(const std::string& maneuver_name,
                               const rclcpp::NodeOptions& options,
                               const std::string& topic_namespace_prefix = "",
                               bool deactivate_before_completion = true,
                               std::chrono::milliseconds execution_interval = DEFAULT_VALUE_EXECUTION_INTERVAL,
                               std::chrono::milliseconds feedback_interval = DEFAULT_VALUE_FEEDBACK_INTERVAL);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

   private:
    rclcpp::Node::SharedPtr node_ptr_;  // It's necessary to also store the node pointer here for successful destruction
    std::unique_ptr<ManeuverMode<ActionT>> maneuver_mode_ptr_;
    std::shared_ptr<FlightModeExecutor<ActionT>> flight_mode_executor_ptr_;
};

template <class ActionT, class ModeT>
ExternalFlightModeExecutor<ActionT, ModeT>::ExternalFlightModeExecutor(const std::string& maneuver_name,
                                                                       const rclcpp::NodeOptions& options,
                                                                       const std::string& topic_namespace_prefix,
                                                                       bool deactivate_before_completion,
                                                                       std::chrono::milliseconds execution_interval,
                                                                       std::chrono::milliseconds feedback_interval)
    : node_ptr_{std::make_shared<rclcpp::Node>("maneuver_" + maneuver_name + "_node", options)}
{
    static_assert(std::is_base_of<ManeuverMode<ActionT>, ModeT>::value,
                  "Template argument ModeT must inherit ManeuverMode<class ActionT> as public and with same type "
                  "ActionT as Maneuver<ActionT>");

    const auto action_context_ptr = std::make_shared<ActionContext<ActionT>>(node_ptr_->get_logger());

    maneuver_mode_ptr_ = std::make_unique<ModeT>(*node_ptr_,
                                                 px4_ros2::ModeBase::Settings{"mode_" + maneuver_name},
                                                 topic_namespace_prefix,
                                                 action_context_ptr);

    if (!px4_ros2::waitForFMU(*node_ptr_, std::chrono::seconds(3))) { throw std::runtime_error("No message from FMU"); }
    else {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "FMU availability test successful.");
    }

    if (!maneuver_mode_ptr_->doRegister()) {
        RCLCPP_FATAL(node_ptr_->get_logger(),
                     "Registration of ManeuverMode for maneuver '%s' failed.",
                     maneuver_name.c_str());
        throw std::runtime_error("Mode registration failed");
    }

    // AFTER (!) registration, the mode id can be queried to set up the executor
    flight_mode_executor_ptr_ = std::make_shared<FlightModeExecutor<ActionT>>(maneuver_name,
                                                                              node_ptr_,
                                                                              action_context_ptr,
                                                                              maneuver_mode_ptr_->id(),
                                                                              deactivate_before_completion,
                                                                              execution_interval,
                                                                              feedback_interval);
}

template <class ActionT, class ModeT>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
ExternalFlightModeExecutor<ActionT, ModeT>::get_node_base_interface()
{
    return node_ptr_->get_node_base_interface();
}

}  // namespace px4_behavior
