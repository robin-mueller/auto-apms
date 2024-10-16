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

#include "auto_apms_behavior_tree/bt_executor.hpp"

#include <chrono>
#include <functional>

#include "auto_apms_behavior_tree/constants.hpp"

#define STATE_CHANGE_LOGGING_PARAM_NAME "state_change_logging"

namespace auto_apms_behavior_tree {

std::string to_string(BTExecutorState state)
{
    switch (state) {
        case BTExecutorState::IDLE:
            return "IDLE";
        case BTExecutorState::RUNNING:
            return "RUNNING";
        case BTExecutorState::PAUSED:
            return "PAUSED";
        case BTExecutorState::HALTED:
            return "HALTED";
        case BTExecutorState::TERMINATED:
            return "TERMINATED";
    }
    return "undefined";
}

std::string to_string(BTExecutorCommand cmd)
{
    switch (cmd) {
        case BTExecutorCommand::RUN:
            return "RUN";
        case BTExecutorCommand::PAUSE:
            return "PAUSE";
        case BTExecutorCommand::HALT:
            return "HALT";
        case BTExecutorCommand::TERMINATE:
            return "TERMINATE";
    }
    return "undefined";
}

BTExecutor::BTExecutor(const std::string &name,
                       const rclcpp::NodeOptions &options,
                       const int groot2_server_port,
                       const std::chrono::milliseconds &bt_tick_interval)
    : node_ptr_{std::make_shared<rclcpp::Node>(name, options)},
      groot2_server_port_{groot2_server_port},
      bt_tick_interval_{bt_tick_interval}
{
#ifdef _AUTO_APMS_BEHAVIOR_TREE_DEBUG_LOGGING
    // Set logging severity to DEBUG
    auto ret = rcutils_logging_set_logger_level(node_ptr_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }
#endif

    auto upload_service_name = name + std::string(BT_EXECUTOR_UPLOAD_SERVICE_NAME_SUFFIX);
    auto launch_action_name = name + std::string(BT_EXECUTOR_LAUNCH_ACTION_NAME_SUFFIX);
    auto command_action_name = name + std::string(BT_EXECUTOR_COMMAND_ACTION_NAME_SUFFIX);

    // State change logging parameter
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "True to log node status changes, false otherwise";
    node_ptr_->declare_parameter(STATE_CHANGE_LOGGING_PARAM_NAME, false, desc);

    param_subscriber_ptr_ = std::make_shared<rclcpp::ParameterEventHandler>(node_ptr_);
    auto cb = [this](const rclcpp::Parameter &p) {
        if (this->bt_state_observer_ptr_) { this->bt_state_observer_ptr_->set_state_change_logging(p.as_bool()); }
    };
    state_change_logging_param_handle_ptr_ =
        param_subscriber_ptr_->add_parameter_callback(STATE_CHANGE_LOGGING_PARAM_NAME, cb);

    using namespace std::placeholders;
    launch_action_ptr_ =
        rclcpp_action::create_server<LaunchExecutorAction>(node_ptr_,
                                                           launch_action_name,
                                                           std::bind(&BTExecutor::LaunchHandleGoal, this, _1, _2),
                                                           std::bind(&BTExecutor::LaunchHandleCancel, this, _1),
                                                           std::bind(&BTExecutor::LaunchHandleAccepted, this, _1));

    upload_service_ptr_ =
        node_ptr_->create_service<UploadBehaviorTreeService>(upload_service_name,
                                                             std::bind(&BTExecutor::UploadBehaviorTree, this, _1, _2));

    command_action_ptr_ =
        rclcpp_action::create_server<CommandAction>(node_ptr_,
                                                    command_action_name,
                                                    std::bind(&BTExecutor::CommandHandleGoal, this, _1, _2),
                                                    std::bind(&BTExecutor::CommandHandleCancel, this, _1),
                                                    std::bind(&BTExecutor::CommandHandleAccepted, this, _1));

    global_blackboard_ptr_ = BT::Blackboard::create();
}

void BTExecutor::OnTreeCreated(BT::Blackboard &) {}

BTExecutor::Command BTExecutor::ReviewControlCommand(Command current_command, State) { return current_command; }

void BTExecutor::BeforeFirstTick(BT::Blackboard &) {}

BTExecutor::ClosureConduct BTExecutor::OnResult(bool) { return ClosureConduct::SUCCEED; }

void BTExecutor::UploadBehaviorTree(const std::shared_ptr<UploadBehaviorTreeService::Request> request,
                                    std::shared_ptr<UploadBehaviorTreeService::Response> response)
{
    // Deny upload if executor is currently RUNNING
    if (GetExecutionState() == State::RUNNING) {
        response->success = false;
        response->error_message = "UploadBehaviorTree DENIED: '" + GetCreatedTreeID() + "' is currently running";
        RCLCPP_ERROR(node_ptr_->get_logger(), "%s", response->error_message.c_str());
        return;
    }

    // Note: It is mandatory to call the pure virtual function SetupBehaviorTreeFactory after the constructor,
    //       so this is done in the process of registering a tree. The factory may change for each request (not
    //       implemented tho)
    bt_factory_ptr_ = std::make_unique<BT::BehaviorTreeFactory>();
    try {
        this->SetupBehaviorTreeFactory(node_ptr_, *this->bt_factory_ptr_);
    } catch (const std::exception &e) {
        response->success = false;
        response->error_message = "Error setting up behavior tree factory: " + std::string(e.what());
        RCLCPP_ERROR(node_ptr_->get_logger(), "%s", response->error_message.c_str());
        return;
    }

    try {
        bt_factory_ptr_->registerBehaviorTreeFromText(request->xml_data);
    } catch (const std::exception &e) {
        response->success = false;
        response->error_message = "Error registering behavior tree: " + std::string(e.what());
        RCLCPP_ERROR(node_ptr_->get_logger(), "%s", response->error_message.c_str());
        return;
    }

    // Log list of currently registered IDs
    std::vector<std::string> registered_ids = bt_factory_ptr_->registeredBehaviorTrees();
    std::ostringstream oss;
    std::copy(registered_ids.begin(), registered_ids.end() - 1, std::ostream_iterator<std::string>(oss, ", "));
    oss << registered_ids.back();
    RCLCPP_DEBUG(node_ptr_->get_logger(), "Tree registration successful - All registered IDs: %s", oss.str().c_str());

    // Try to create tree
    try {
        CreateTree(request->tree_id);
    } catch (const std::exception &e) {
        response->success = false;
        response->error_message = "Error creating tree with ID '" + request->tree_id + "': " + std::string(e.what());
        RCLCPP_ERROR(node_ptr_->get_logger(), "%s", response->error_message.c_str());
        return;
    }
    OnTreeCreated(*global_blackboard_ptr_);

    response->success = true;
}

rclcpp_action::GoalResponse BTExecutor::LaunchHandleGoal(const rclcpp_action::GoalUUID &uuid,
                                                         std::shared_ptr<const LaunchExecutorAction::Goal> goal_ptr)
{
    (void)goal_ptr;

    // Reject if a tree is already executing
    if (GetExecutionState() != State::TERMINATED) {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "Goal %s was REJECTED because a tree with ID '%s' is already executing",
                    rclcpp_action::to_string(uuid).c_str(),
                    GetCreatedTreeID().c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!behavior_tree_ptr_) {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "Goal %s was REJECTED because no tree has been created yet",
                    rclcpp_action::to_string(uuid).c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    SetUpExecutionRoutine();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BTExecutor::LaunchHandleCancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<LaunchExecutorAction>> goal_handle_ptr)
{
    (void)goal_handle_ptr;
    control_command_ = Command::HALT;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BTExecutor::LaunchHandleAccepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<LaunchExecutorAction>> goal_handle_ptr)
{
    // Assign weak ptr for launch
    launch_goal_handle_ptr_ = goal_handle_ptr;

    // Launch executor by instantiating a timer
    execution_timer_ptr_ = node_ptr_->create_wall_timer(
        bt_tick_interval_,
        [this, goal_handle_ptr, action_result_ptr = std::make_shared<LaunchExecutorAction::Result>()]() {
            auto current_tree_id = GetCreatedTreeID();  // May change on new upload
            action_result_ptr->terminated_tree_id = current_tree_id;
            action_result_ptr->tree_result = LaunchExecutorAction::Result::TREE_RESULT_NOT_SET;

            auto abort_goal = [this, goal_handle_ptr, action_result_ptr](const std::string &msg) {
                action_result_ptr->termination_message =
                    msg + "\nProcess terminates from state " + to_string(GetExecutionState());
                RCLCPP_ERROR(node_ptr_->get_logger(),
                             "Behavior tree execution aborted with message: %s",
                             action_result_ptr->termination_message.c_str());
                goal_handle_ptr->abort(action_result_ptr);
                execution_timer_ptr_->cancel();
            };

            // Handle cancellation
            if (goal_handle_ptr->is_canceling() && GetExecutionState() == State::HALTED) {
                RCLCPP_INFO(node_ptr_->get_logger(), "Behavior tree execution was canceled successfully");
                goal_handle_ptr->canceled(action_result_ptr);
                execution_timer_ptr_->cancel();
                return;
            }

            auto running_action_history = bt_state_observer_ptr_->running_action_history();
            if (!running_action_history.empty()) {
                // If there are multiple nodes running, join the IDs to a single string
                std::ostringstream oss;
                std::copy(running_action_history.begin(),
                          running_action_history.end() - 1,
                          std::ostream_iterator<std::string>(oss, " + "));
                oss << running_action_history.back();

                // Reset the history cache
                bt_state_observer_ptr_->flush();

                last_feedback_.running_action_name = oss.str();
                last_feedback_.running_action_timestamp =
                    std::chrono::duration<double>{std::chrono::high_resolution_clock::now().time_since_epoch()}.count();
            }

            auto execution_state_before = GetExecutionState();

            // Publish feedback before control command is evaluated
            auto feedback_ptr = std::make_shared<LaunchExecutorAction::Feedback>();
            feedback_ptr->execution_state_str = to_string(execution_state_before);
            feedback_ptr->root_tree_id = current_tree_id;
            feedback_ptr->running_action_name = last_feedback_.running_action_name;
            feedback_ptr->running_action_timestamp = last_feedback_.running_action_timestamp;
            goal_handle_ptr->publish_feedback(feedback_ptr);

            is_paused_ = false;
            switch (ReviewControlCommand(control_command_, execution_state_before)) {
                case Command::RUN:
                    break;
                case Command::PAUSE:
                    is_paused_ = true;
                    return;
                case Command::TERMINATE:
                    if (execution_state_before == State::HALTED) {
                        abort_goal("Termination due to internal control command");
                        return;
                    }
                    // Fall through to halt tree before termination
                    [[fallthrough]];
                case Command::HALT:
                    // Check if already halted
                    if (execution_state_before == State::HALTED) { return; }
                    try {
                        behavior_tree_ptr_->haltTree();
                        return;
                    } catch (const std::exception &e) {
                        abort_goal("Error during haltTree() on command " + to_string(control_command_) + ": " +
                                   std::string(e.what()));
                        return;
                    }
                    break;
                default:
                    throw std::logic_error("Handling of control command " +
                                           std::to_string(static_cast<int>(control_command_)) + " '" +
                                           to_string(control_command_) + "' is not implemented");
            }

            // Evaluate callback before executor ticks tree for the first time
            if (execution_state_before == State::IDLE) { BeforeFirstTick(*global_blackboard_ptr_); }

            BT::NodeStatus bt_status = BT::NodeStatus::IDLE;
            try {
                // It is important to tick EXACTLY once,
                // since we must prevent blocking the ROS2 runtime execution stack for too long
                bt_status = behavior_tree_ptr_->tickExactlyOnce();
            } catch (const std::exception &e) {
                std::string msg = "Behavior tree ran into an exception during tick: " + std::string(e.what());

                // Try to halt tree before aborting
                try {
                    behavior_tree_ptr_->haltTree();
                } catch (const std::exception &e) {
                    msg += "\nDuring haltTree(), another error occured: " + std::string(e.what());
                }
                abort_goal(msg);
                return;
            }

            // Determine if routine is to continue
            if (bt_status == BT::NodeStatus::RUNNING) { return; }

            switch (bt_status) {
                case BT::NodeStatus::SUCCESS:
                    action_result_ptr->tree_result = LaunchExecutorAction::Result::TREE_RESULT_SUCCESS;
                    break;
                case BT::NodeStatus::FAILURE:
                    action_result_ptr->tree_result = LaunchExecutorAction::Result::TREE_RESULT_FAILURE;
                    break;
                default:
                    action_result_ptr->tree_result = LaunchExecutorAction::Result::TREE_RESULT_NOT_SET;
                    break;
            }

            // Determine how to deal with tree termination with respect to the execution routine
            switch (OnResult(bt_status == BT::NodeStatus::SUCCESS)) {
                case ClosureConduct::SUCCEED:
                    goal_handle_ptr->succeed(action_result_ptr);
                    execution_timer_ptr_->cancel();
                    return;
                case ClosureConduct::ABORT:
                    abort_goal("Behavior tree " + current_tree_id + " finished with status " + BT::toStr(bt_status));
                    return;
                case ClosureConduct::RESTART:
                    SetUpExecutionRoutine();
                    return;
            }

            throw std::logic_error("Execution routine is not designed to proceed to this statement");
        });
}

rclcpp_action::GoalResponse BTExecutor::CommandHandleGoal(const rclcpp_action::GoalUUID &uuid,
                                                          std::shared_ptr<const CommandAction::Goal> goal_ptr)
{
    (void)uuid;

    if (command_request_timer_ptr_ && !command_request_timer_ptr_->is_canceled()) {
        RCLCPP_WARN(node_ptr_->get_logger(), "Command rejected, because previous request is busy");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (HasLaunched() && launch_goal_handle_ptr_.lock()->is_canceling()) {
        RCLCPP_WARN(node_ptr_->get_logger(), "Command rejected, because executor is canceling");
        return rclcpp_action::GoalResponse::REJECT;
    }

    auto execution_state = GetExecutionState();
    switch (goal_ptr->command) {
        case CommandAction::Goal::COMMAND_RESUME:
            if (execution_state == State::PAUSED || execution_state == State::HALTED) {
                RCLCPP_INFO(node_ptr_->get_logger(), "Tree with ID '%s' will RESUME", GetCreatedTreeID().c_str());
            }
            else {
                RCLCPP_INFO(node_ptr_->get_logger(),
                            "Requested to RESUME while executor is in state %s. Will do anyways",
                            to_string(execution_state).c_str());
            }
            break;
        case CommandAction::Goal::COMMAND_PAUSE:
            if (execution_state == State::RUNNING) {
                RCLCPP_INFO(node_ptr_->get_logger(), "Tree with ID '%s' will PAUSE", GetCreatedTreeID().c_str());
            }
            else {
                RCLCPP_INFO(node_ptr_->get_logger(),
                            "Requested to PAUSE while executor is in state %s. Will do anyways",
                            to_string(execution_state).c_str());
            }
            break;
        case CommandAction::Goal::COMMAND_HALT:
            if (execution_state == State::RUNNING || execution_state == State::PAUSED) {
                RCLCPP_INFO(node_ptr_->get_logger(), "Tree with ID '%s' will HALT", GetCreatedTreeID().c_str());
            }
            else {
                RCLCPP_INFO(node_ptr_->get_logger(),
                            "Requested to HALT while executor is in state %s. Will do anyways",
                            to_string(execution_state).c_str());
            }
            break;
        case CommandAction::Goal::COMMAND_TERMINATE:
            if (HasLaunched()) {
                RCLCPP_INFO(node_ptr_->get_logger(), "Behavior executor %s will TERMINATE", node_ptr_->get_name());
            }
            else {
                RCLCPP_INFO(node_ptr_->get_logger(),
                            "Requested to TERMINATE although execution process has not launched yet. Will do anyways");
            }
            break;
        default:
            RCLCPP_WARN(node_ptr_->get_logger(), "Command %i is undefined. Rejecting request", goal_ptr->command);
            return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BTExecutor::CommandHandleCancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<CommandAction>> goal_handle_ptr)
{
    (void)goal_handle_ptr;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BTExecutor::CommandHandleAccepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<CommandAction>> goal_handle_ptr)
{
    // Set control command
    auto prev_command = control_command_;
    switch (goal_handle_ptr->get_goal()->command) {
        case CommandAction::Goal::COMMAND_RESUME:
            control_command_ = Command::RUN;
            break;
        case CommandAction::Goal::COMMAND_PAUSE:
            control_command_ = Command::PAUSE;
            break;
        case CommandAction::Goal::COMMAND_HALT:
            control_command_ = Command::HALT;
            break;
        case CommandAction::Goal::COMMAND_TERMINATE:
            control_command_ = Command::TERMINATE;
            break;
    }

    command_request_timer_ptr_ = node_ptr_->create_wall_timer(
        bt_tick_interval_,
        [this, goal_handle_ptr, prev_command, action_result_ptr = std::make_shared<CommandAction::Result>()]() {
            // Check if canceling
            if (goal_handle_ptr->is_canceling()) {
                // We don't know how far process proceeded, but the least we can do is reset the flag
                control_command_ = prev_command;
                goal_handle_ptr->canceled(action_result_ptr);
                command_request_timer_ptr_->cancel();
                return;
            }

            // If the execution timer canceled during goal, it failed
            if (control_command_ != BTExecutorCommand::TERMINATE && execution_timer_ptr_->is_canceled()) {
                RCLCPP_ERROR(node_ptr_->get_logger(),
                             "Command %s failed due to cancelation of executon timer. Aborting...",
                             to_string(control_command_).c_str());
                goal_handle_ptr->abort(action_result_ptr);
                command_request_timer_ptr_->cancel();
                return;
            }

            // We should wait for the requested state to be reached
            auto current_state = GetExecutionState();
            switch (control_command_) {
                case Command::RUN:
                    if (current_state != State::RUNNING) { return; }
                    break;
                case Command::PAUSE:
                    if (current_state != State::PAUSED) { return; }
                    break;
                case Command::HALT:
                    if (current_state != State::HALTED) { return; }
                    break;
                case Command::TERMINATE:
                    if (current_state != State::TERMINATED) { return; }
                    break;
            }

            goal_handle_ptr->succeed(action_result_ptr);
            command_request_timer_ptr_->cancel();
        });
}

void BTExecutor::CreateTree(const std::string &tree_id)
{
    // Throw if bt factory hasn't been set up yet
    if (!bt_factory_ptr_) { throw std::runtime_error("Tree factory hasn't been set up yet"); }

    auto new_tree = std::make_unique<BT::Tree>(
        bt_factory_ptr_->createTree(tree_id, BT::Blackboard::create(global_blackboard_ptr_)));

    // If creating tree didn't throw, assign member
    behavior_tree_ptr_ = std::move(new_tree);

    // Set up Groot2 publisher
    bt_groot2_publisher_ptr_.reset();
    bt_groot2_publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(*behavior_tree_ptr_, groot2_server_port_);

    // Set up tree state observer
    bt_state_observer_ptr_ = std::make_unique<BTStateObserver>(*behavior_tree_ptr_, node_ptr_->get_logger());
    bt_state_observer_ptr_->enableTransitionToIdle(false);
    state_change_logging_param_handle_ptr_->callback(node_ptr_->get_parameter(STATE_CHANGE_LOGGING_PARAM_NAME));

    RCLCPP_INFO(node_ptr_->get_logger(), "Tree '%s' was created successfully", GetCreatedTreeID().c_str());
}

BTExecutor::State BTExecutor::GetExecutionState()
{
    if (HasLaunched()) {
        if (behavior_tree_ptr_->rootNode()->status() == BT::NodeStatus::IDLE) {
            // If the root node is IDLE
            return is_paused_ ? State::IDLE : State::HALTED;
        }
        return is_paused_ ? State::PAUSED : State::RUNNING;
    }
    return State::TERMINATED;
}

std::string BTExecutor::GetCreatedTreeID()
{
    if (!behavior_tree_ptr_) {
        throw std::runtime_error("Error getting executing tree id: behavior_tree_ptr_ is nullptr");
    }
    std::string tree_id = behavior_tree_ptr_->subtrees[0]->tree_ID;
    if (tree_id.empty()) {
        throw std::runtime_error("Error getting executing tree id: tree_id is not allowed to be empty");
    }
    return tree_id;
}

bool BTExecutor::HasLaunched() { return execution_timer_ptr_ && !execution_timer_ptr_->is_canceled(); }

void BTExecutor::SetUpExecutionRoutine()
{
    is_paused_ = true;
    control_command_ = Command::RUN;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr BTExecutor::get_node_base_interface()
{
    return node_ptr_->get_node_base_interface();
}

rclcpp::Node::SharedPtr BTExecutor::node() { return node_ptr_; }
BT::Blackboard::Ptr BTExecutor::global_blackboard() { return global_blackboard_ptr_; }

}  // namespace auto_apms_behavior_tree
