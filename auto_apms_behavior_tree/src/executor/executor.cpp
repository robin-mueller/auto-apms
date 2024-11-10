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

#include "auto_apms_behavior_tree/executor/executor.hpp"

#include <chrono>

#include "auto_apms_util/logging.hpp"

namespace auto_apms_behavior_tree
{

TreeExecutor::TreeExecutor(rclcpp::Node::SharedPtr node_ptr)
: node_ptr_(node_ptr),
  global_blackboard_ptr_(TreeBlackboard::create()),
  control_command_(ControlCommand::RUN),
  execution_stopped_(true)
{
  auto_apms_util::exposeToDebugLogging(node_ptr_->get_logger());
}

std::shared_future<TreeExecutor::ExecutionResult> TreeExecutor::startExecution(
  TreeConstructor make_tree, double tick_rate_sec, unsigned int groot2_port)
{
  if (isBusy()) {
    throw exceptions::TreeExecutionError(
      "Cannot start execution with tree '" + getTreeName() + "' currently executing.");
  }

  try {
    tree_ptr_.reset(new Tree(make_tree(global_blackboard_ptr_)));
  } catch (const std::exception & e) {
    throw exceptions::TreeBuildError(
      "Cannot start execution because creating the tree failed: " + std::string(e.what()));
  }

  groot2_publisher_ptr_.reset();
  groot2_publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(*tree_ptr_, groot2_port);
  state_observer_ptr_.reset();
  state_observer_ptr_ = std::make_unique<TreeStateObserver>(*tree_ptr_, node_ptr_->get_logger());
  state_observer_ptr_->enableTransitionToIdle(false);

  /* Start execution timer */

  // Reset state variables
  prev_execution_state_ = getExecutionState();
  control_command_ = ControlCommand::RUN;
  termination_reason_ = "";
  execution_stopped_ = true;

  // Create promise for asnychronous execution and configure termination callback
  auto promise_ptr = std::make_shared<std::promise<ExecutionResult>>();
  TerminationCallback termination_callback = [this, promise_ptr](ExecutionResult result, const std::string & msg) {
    RCLCPP_INFO(
      node_ptr_->get_logger(), "Terminating behavior tree execution from state %s. Reason: %s.",
      toStr(getExecutionState()).c_str(), msg.c_str());
    onTermination(result);  // is evaluated before the timer is cancelled, which means the execution state has not
                            // changed yet during the callback and can be evaluated to inspect the terminal state.
    promise_ptr->set_value(result);
    execution_timer_ptr_->cancel();
    tree_ptr_.reset();  // Release the memory allocated by the tree
  };

  execution_timer_ptr_ = node_ptr_->create_wall_timer(
    std::chrono::duration<double>(tick_rate_sec),
    [this, termination_callback]() { execution_routine_(termination_callback); });
  return promise_ptr->get_future();
}

bool TreeExecutor::isBusy() { return execution_timer_ptr_ && !execution_timer_ptr_->is_canceled(); }

TreeExecutor::ExecutionState TreeExecutor::getExecutionState()
{
  if (isBusy()) {
    if (!tree_ptr_) throw std::logic_error("tree_ptr_ cannot be nullptr when execution is started.");
    if (tree_ptr_->rootNode()->status() == BT::NodeStatus::IDLE) {
      // The root node being IDLE here means that the tree hasn't been ticked yet since its creation or was halted
      return execution_stopped_ ? ExecutionState::STARTING : ExecutionState::HALTED;
    }
    return execution_stopped_ ? ExecutionState::PAUSED : ExecutionState::RUNNING;
  }
  return ExecutionState::IDLE;
}

std::string TreeExecutor::getTreeName()
{
  if (tree_ptr_) return tree_ptr_->subtrees[0]->tree_ID;
  return "NO_TREE_NAME";
}

void TreeExecutor::execution_routine_(TerminationCallback termination_callback)
{
  const ExecutionState this_execution_state = getExecutionState();
  if (prev_execution_state_ != this_execution_state) {
    RCLCPP_DEBUG(
      node_ptr_->get_logger(), "Executor for tree '%s' changed state from '%s' to '%s'.", getTreeName().c_str(),
      toStr(prev_execution_state_).c_str(), toStr(this_execution_state).c_str());
    prev_execution_state_ = this_execution_state;
  }

  /* Evaluate control command */

  execution_stopped_ = false;
  bool do_on_tick = true;
  switch (control_command_) {
    case ControlCommand::PAUSE:
      execution_stopped_ = true;
      return;
    case ControlCommand::RUN:
      if (this_execution_state == ExecutionState::STARTING) {
        // Evaluate initial tick callback before ticking for the first time since the timer has been created
        if (!onInitialTick()) {
          do_on_tick = false;
          termination_reason_ = "onInitialTick() returned false";
        }
      }
      // Evaluate tick callback everytime before actually ticking.
      // This also happens the first time except if onInitialTick() returned false
      if (do_on_tick) {
        if (onTick()) {
          break;
        } else {
          termination_reason_ = "onTick() returned false";
        }
      }

      // Fall through to terminate if any of the callbacks returned false
      [[fallthrough]];
    case ControlCommand::TERMINATE:
      if (this_execution_state == ExecutionState::HALTED) {
        termination_callback(
          ExecutionResult::TERMINATED_PREMATURELY,
          termination_reason_.empty() ? "Terminated by control command" : termination_reason_);
        return;
      }

      // Fall through to halt tree before termination
      [[fallthrough]];
    case ControlCommand::HALT:
      // Check if already halted
      if (this_execution_state != ExecutionState::HALTED) {
        try {
          tree_ptr_->haltTree();
        } catch (const std::exception & e) {
          termination_callback(
            ExecutionResult::ERROR,
            "Error during haltTree() on command " + toStr(control_command_) + ": " + std::string(e.what()));
        }
      }
      return;
    default:
      throw std::logic_error(
        "Handling control command " + std::to_string(static_cast<int>(control_command_)) + " '" +
        toStr(control_command_) + "' is not implemented.");
  }

  /* Tick the tree instance */

  BT::NodeStatus bt_status = BT::NodeStatus::IDLE;
  try {
    // It is important to tick EXACTLY once to prevent loops induced by BT nodes from blocking
    bt_status = tree_ptr_->tickExactlyOnce();
  } catch (const std::exception & e) {
    std::string msg = "Ran into an exception during tick: " + std::string(e.what());
    try {
      tree_ptr_->haltTree();  // Try to halt tree before aborting
    } catch (const std::exception & e) {
      msg += "\nDuring haltTree(), another exception occured: " + std::string(e.what());
    }
    termination_callback(ExecutionResult::ERROR, msg);
    return;
  }

  /* Continue execution until tree is not running anymore */

  if (bt_status == BT::NodeStatus::RUNNING) return;

  /* Determine how to handle the behavior tree execution result */

  if (!(bt_status == BT::NodeStatus::SUCCESS || bt_status == BT::NodeStatus::FAILURE)) {
    throw std::logic_error(
      "bt_status is " + BT::toStr(bt_status) + ". Must be one of SUCCESS or FAILURE at this point.");
  }
  const bool success = bt_status == BT::NodeStatus::SUCCESS;
  switch (onTreeExit(success)) {
    case TreeExitBehavior::TERMINATE:
      termination_callback(
        success ? ExecutionResult::TREE_SUCCEEDED : ExecutionResult::TREE_FAILED,
        "Terminated on tree result " + BT::toStr(bt_status));
      return;
    case TreeExitBehavior::RESTART:
      control_command_ = ControlCommand::RUN;
      return;
  }

  throw std::logic_error("Execution routine is not intended to proceed to this statement.");
}

bool TreeExecutor::onInitialTick() { return true; }

bool TreeExecutor::onTick() { return true; }

TreeExecutor::TreeExitBehavior TreeExecutor::onTreeExit(bool /*success*/) { return TreeExitBehavior::TERMINATE; }

void TreeExecutor::onTermination(const ExecutionResult & /*result*/) {}

void TreeExecutor::setControlCommand(ControlCommand cmd) { control_command_ = cmd; }

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr TreeExecutor::get_node_base_interface()
{
  return node_ptr_->get_node_base_interface();
}

TreeBlackboardSharedPtr TreeExecutor::getGlobalBlackboardPtr() { return global_blackboard_ptr_; }

TreeStateObserver & TreeExecutor::getStateObserver() { return *state_observer_ptr_; }

std::string toStr(TreeExecutor::ExecutionState state)
{
  switch (state) {
    case TreeExecutor::ExecutionState::IDLE:
      return "IDLE";
    case TreeExecutor::ExecutionState::STARTING:
      return "STARTING";
    case TreeExecutor::ExecutionState::RUNNING:
      return "RUNNING";
    case TreeExecutor::ExecutionState::PAUSED:
      return "PAUSED";
    case TreeExecutor::ExecutionState::HALTED:
      return "HALTED";
  }
  return "undefined";
}

std::string toStr(TreeExecutor::ControlCommand cmd)
{
  switch (cmd) {
    case TreeExecutor::ControlCommand::RUN:
      return "RUN";
    case TreeExecutor::ControlCommand::PAUSE:
      return "PAUSE";
    case TreeExecutor::ControlCommand::HALT:
      return "HALT";
    case TreeExecutor::ControlCommand::TERMINATE:
      return "TERMINATE";
  }
  return "undefined";
}

std::string toStr(TreeExecutor::TreeExitBehavior behavior)
{
  switch (behavior) {
    case TreeExecutor::TreeExitBehavior::TERMINATE:
      return "TERMINATE";
    case TreeExecutor::TreeExitBehavior::RESTART:
      return "RESTART";
  }
  return "undefined";
}

std::string toStr(TreeExecutor::ExecutionResult result)
{
  switch (result) {
    case TreeExecutor::ExecutionResult::TREE_SUCCEEDED:
      return "TREE_SUCCEEDED";
    case TreeExecutor::ExecutionResult::TREE_FAILED:
      return "TREE_FAILED";
    case TreeExecutor::ExecutionResult::TERMINATED_PREMATURELY:
      return "TERMINATED_PREMATURELY";
    case TreeExecutor::ExecutionResult::ERROR:
      return "ERROR";
  }
  return "undefined";
}

}  // namespace auto_apms_behavior_tree
