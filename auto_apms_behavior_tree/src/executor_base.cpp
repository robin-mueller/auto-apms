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

#include "auto_apms_behavior_tree/executor_base.hpp"

#include <chrono>

namespace auto_apms_behavior_tree {

BTExecutorBase::BTExecutorBase(rclcpp::Node::SharedPtr node_ptr)
    : node_ptr_{node_ptr},
      global_blackboard_ptr_{BT::Blackboard::create()},
      control_command_{ControlCommand::RUN},
      execution_stopped_{true}
{}

void BTExecutorBase::CreateTree(BTCreator &tree_creator, const std::string &main_tree_id)
{
    if (IsStarted()) throw std::logic_error("Cannot create tree while execution routine is running.");
    tree_ptr_.reset();
    tree_ptr_ = std::make_unique<BT::Tree>(tree_creator.CreateTree(main_tree_id, global_blackboard_ptr_));
}

std::shared_future<BTExecutorBase::ClosureResult> BTExecutorBase::Start()
{
    if (!tree_ptr_) throw std::logic_error("Cannot start execution because tree hasn't been created yet.");

    groot2_publisher_ptr_.reset();
    groot2_publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(*tree_ptr_, executor_params_.groot2_port);
    state_observer_ptr_.reset();
    state_observer_ptr_ = std::make_unique<BTStateObserver>(*tree_ptr_, node_ptr_->get_logger());
    state_observer_ptr_->enableTransitionToIdle(false);

    /* Start execution timer */

    // Reset state variables
    control_command_ = ControlCommand::RUN;
    termination_reason_ = "";

    // Create promise for asnychronous execution
    auto promise_ptr = std::make_shared<std::promise<ClosureResult>>();
    CloseExecutionCallback cb = [this, promise_ptr](ClosureResult result, const std::string &msg) {
        RCLCPP_DEBUG(node_ptr_->get_logger(),
                     "Closing behavior tree execution from state %s. Reason: %s.",
                     to_string(GetExecutionState()).c_str(),
                     msg.c_str());
        execution_timer_ptr_->cancel();
        promise_ptr->set_value(result);
        OnClose();
    };
    execution_timer_ptr_ =
        node_ptr_->create_wall_timer(std::chrono::duration<double, std::milli>(executor_params_.tick_rate),
                                     [this, cb]() { ExecutionRoutine(cb); });

    return promise_ptr->get_future();
}

bool BTExecutorBase::IsStarted() { return execution_timer_ptr_ && !execution_timer_ptr_->is_canceled(); }

BTExecutorBase::ExecutionState BTExecutorBase::GetExecutionState()
{
    if (IsStarted()) {
        if (!tree_ptr_) throw std::logic_error("Cannot get execution state because tree_ptr_ is nullptr");
        if (tree_ptr_->rootNode()->status() == BT::NodeStatus::IDLE) {
            // The root node being IDLE means that the tree hasn't been ticked yet since its creation or was halted
            return execution_stopped_ ? ExecutionState::IDLE : ExecutionState::HALTED;
        }
        return execution_stopped_ ? ExecutionState::PAUSED : ExecutionState::RUNNING;
    }
    return ExecutionState::TERMINATED;
}

void BTExecutorBase::ExecutionRoutine(CloseExecutionCallback close_callback)
{
    /* Evaluate control command */

    const ExecutionState execution_state_before = GetExecutionState();  // Freeze current execution state
    execution_stopped_ = false;
    switch (control_command_) {
        case ControlCommand::PAUSE:
            execution_stopped_ = true;
            return;
        case ControlCommand::RUN:
            // Evaluate OnFirstTick callback before executor ticks tree for the first time
            if (execution_state_before == ExecutionState::IDLE && OnFirstTick()) {
                termination_reason_ = "OnFirstTick returned false";
                break;
            }

            // Evaluate OnTick callback
            if (OnTick()) {
                termination_reason_ = "OnTick returned false";
                break;
            }

            // Fall through if any of the callbacks returned false to terminate
            [[fallthrough]];
        case ControlCommand::TERMINATE:
            if (execution_state_before == ExecutionState::HALTED) {
                close_callback(ClosureResult::TERMINATED_PREMATURELY,
                               termination_reason_.empty() ? "Terminated by control command" : termination_reason_);
                return;
            }

            // Fall through to halt tree before termination
            [[fallthrough]];
        case ControlCommand::HALT:
            // Check if already halted
            if (execution_state_before != ExecutionState::HALTED) {
                try {
                    tree_ptr_->haltTree();
                } catch (const std::exception &e) {
                    close_callback(ClosureResult::ERROR,
                                   "Error during haltTree() on command " + to_string(control_command_) + ": " +
                                       std::string(e.what()));
                }
            }
            return;
        default:
            throw std::logic_error("Handling of control command " + std::to_string(static_cast<int>(control_command_)) +
                                   " '" + to_string(control_command_) + "' is not implemented.");
    }

    /* Tick the BT::Tree instance */

    state_observer_ptr_->set_logging(executor_params_.state_change_logger);
    BT::NodeStatus bt_status = BT::NodeStatus::IDLE;
    try {
        // It is important to tick EXACTLY once to prevent loops induced by BT nodes from blocking
        bt_status = tree_ptr_->tickExactlyOnce();
    } catch (const std::exception &e) {
        std::string msg = "Ran into an exception during tick: " + std::string(e.what());
        try {
            tree_ptr_->haltTree();  // Try to halt tree before aborting
        } catch (const std::exception &e) {
            msg += "\nDuring haltTree(), another exception occured: " + std::string(e.what());
        }
        close_callback(ClosureResult::ERROR, msg);
        return;
    }

    /* Continue execution until tree is not running anymore */

    if (bt_status == BT::NodeStatus::RUNNING) return;

    /* Determine how to handle the behavior tree execution result */

    if (!(bt_status == BT::NodeStatus::SUCCESS || bt_status == BT::NodeStatus::FAILURE)) {
        throw std::logic_error("bt_status is " + BT::toStr(bt_status) +
                               ". Must be one of SUCCESS or FAILURE at this point.");
    }
    const bool success = bt_status == BT::NodeStatus::SUCCESS;
    switch (OnTreeExit(success)) {
        case TreeExitBehavior::CLOSE:
            close_callback(success ? ClosureResult::TREE_SUCCEEDED : ClosureResult::TREE_FAILED,
                           "Closed on tree result " + BT::toStr(bt_status));
            return;
        case TreeExitBehavior::RESTART:
            control_command_ = ControlCommand::RUN;
            return;
    }

    throw std::logic_error("ExecutionRoutine is not intended to proceed to this statement.");
}

bool BTExecutorBase::OnFirstTick() { return true; }

bool BTExecutorBase::OnTick() { return true; }

BTExecutorBase::TreeExitBehavior BTExecutorBase::OnTreeExit(bool /*success*/) { return TreeExitBehavior::CLOSE; }

void BTExecutorBase::OnClose() {}

void BTExecutorBase::set_control_command(ControlCommand cmd) { control_command_ = cmd; }

void BTExecutorBase::set_executor_parameters(const ExecutorParams &p) { executor_params_ = p; }

rclcpp::Node::SharedPtr BTExecutorBase::node() { return node_ptr_; }

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr BTExecutorBase::get_node_base_interface()
{
    return node_ptr_->get_node_base_interface();
}

BT::Blackboard::Ptr BTExecutorBase::global_blackboard() { return global_blackboard_ptr_; }

BTExecutorBase::ExecutorParams BTExecutorBase::executor_parameters() { return executor_params_; }

const BTStateObserver &BTExecutorBase::state_observer() { return *state_observer_ptr_; }

std::string to_string(BTExecutorBase::ExecutionState state)
{
    switch (state) {
        case BTExecutorBase::ExecutionState::IDLE:
            return "IDLE";
        case BTExecutorBase::ExecutionState::RUNNING:
            return "RUNNING";
        case BTExecutorBase::ExecutionState::PAUSED:
            return "PAUSED";
        case BTExecutorBase::ExecutionState::HALTED:
            return "HALTED";
        case BTExecutorBase::ExecutionState::TERMINATED:
            return "TERMINATED";
    }
    return "undefined";
}

std::string to_string(BTExecutorBase::ControlCommand cmd)
{
    switch (cmd) {
        case BTExecutorBase::ControlCommand::RUN:
            return "RUN";
        case BTExecutorBase::ControlCommand::PAUSE:
            return "PAUSE";
        case BTExecutorBase::ControlCommand::HALT:
            return "HALT";
        case BTExecutorBase::ControlCommand::TERMINATE:
            return "TERMINATE";
    }
    return "undefined";
}

std::string to_string(BTExecutorBase::TreeExitBehavior behavior)
{
    switch (behavior) {
        case BTExecutorBase::TreeExitBehavior::CLOSE:
            return "CLOSE";
        case BTExecutorBase::TreeExitBehavior::RESTART:
            return "RESTART";
    }
    return "undefined";
}

std::string to_string(BTExecutorBase::ClosureResult result)
{
    switch (result) {
        case BTExecutorBase::ClosureResult::TREE_SUCCEEDED:
            return "TREE_SUCCEEDED";
        case BTExecutorBase::ClosureResult::TREE_FAILED:
            return "TREE_FAILED";
        case BTExecutorBase::ClosureResult::TERMINATED_PREMATURELY:
            return "TERMINATED_PREMATURELY";
        case BTExecutorBase::ClosureResult::ERROR:
            return "ERROR";
    }
    return "undefined";
}

}  // namespace auto_apms_behavior_tree
