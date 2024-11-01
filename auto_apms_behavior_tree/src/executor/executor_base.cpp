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

#include "auto_apms_behavior_tree/executor/executor_base.hpp"

#include <chrono>

#include "auto_apms_core/logging.hpp"

namespace auto_apms_behavior_tree
{

TreeExecutorBase::TreeExecutorBase(rclcpp::Node::SharedPtr node_ptr)
  : node_ptr_{ node_ptr }
  , global_blackboard_ptr_{ TreeBlackboard::create() }
  , control_command_{ ControlCommand::RUN }
  , execution_stopped_{ true }
{
  auto_apms_core::exposeToDebugLogging(node_ptr_->get_logger());
}

std::shared_future<TreeExecutorBase::ExecutionResult>
TreeExecutorBase::startExecution(CreateTreeCallback create_tree_cb)
{
  auto promise_ptr = std::make_shared<std::promise<ExecutionResult>>();
  if (isBusy())
  {
    RCLCPP_WARN(node_ptr_->get_logger(), "Rejecting execution request, because executor is still busy.");
    promise_ptr->set_value(ExecutionResult::START_REJECTED);
    return promise_ptr->get_future();
  }

  try
  {
    tree_ptr_ = std::make_unique<Tree>(create_tree_cb(global_blackboard_ptr_));
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(node_ptr_->get_logger(), "Rejecting execution request, because creating the tree failed: %s", e.what());
    promise_ptr->set_value(ExecutionResult::START_REJECTED);
    return promise_ptr->get_future();
  }

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
  CloseExecutionCallback close_execution_cb = [this, promise_ptr](ExecutionResult result, const std::string& msg) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "Closing behavior tree execution from state %s. Reason: %s.",
                 toStr(getExecutionState()).c_str(), msg.c_str());
    onClose(result);  // is evaluated before the timer is cancelled, which means the execution state has not changed yet
                      // during the callback and can be evaluated to inspect the terminal state.
    promise_ptr->set_value(result);
    execution_timer_ptr_->cancel();
  };

  execution_timer_ptr_ =
      node_ptr_->create_wall_timer(std::chrono::duration<double, std::milli>(executor_params_.tick_rate),
                                   [this, close_execution_cb]() { execution_routine_(close_execution_cb); });

  return promise_ptr->get_future();
}

bool TreeExecutorBase::isBusy()
{
  return execution_timer_ptr_ && !execution_timer_ptr_->is_canceled();
}

TreeExecutorBase::ExecutionState TreeExecutorBase::getExecutionState()
{
  if (isBusy())
  {
    if (!tree_ptr_)
      throw std::logic_error("tree_ptr_ cannot be nullptr when execution is started.");
    if (tree_ptr_->rootNode()->status() == BT::NodeStatus::IDLE)
    {
      // The root node being IDLE here means that the tree hasn't been ticked yet since its creation or was halted
      return execution_stopped_ ? ExecutionState::STARTING : ExecutionState::HALTED;
    }
    return execution_stopped_ ? ExecutionState::PAUSED : ExecutionState::RUNNING;
  }
  return ExecutionState::IDLE;
}

std::string TreeExecutorBase::getTreeName()
{
  if (tree_ptr_)
    return tree_ptr_->subtrees[0]->tree_ID;
  return "NO_TREE_NAME";
}

void TreeExecutorBase::execution_routine_(CloseExecutionCallback close_execution_cb)
{
  const ExecutionState execution_state_before = getExecutionState();  // Freeze execution state before anything else

  /* Evaluate control command */

  execution_stopped_ = false;
  bool do_on_tick = true;
  switch (control_command_)
  {
    case ControlCommand::PAUSE:
      execution_stopped_ = true;
      return;
    case ControlCommand::RUN:
      if (execution_state_before == ExecutionState::STARTING)
      {
        // Evaluate initial tick callback before ticking for the first time since the timer has been created
        if (!onInitialTick())
        {
          do_on_tick = false;
          termination_reason_ = "onInitialTick() returned false";
        }
      }
      // Evaluate tick callback everytime before actually ticking.
      // This also happens the first time except if onInitialTick() returned false
      if (do_on_tick)
      {
        if (onTick())
        {
          break;
        }
        else
        {
          termination_reason_ = "onTick() returned false";
        }
      }

      // Fall through to terminate if any of the callbacks returned false
      [[fallthrough]];
    case ControlCommand::TERMINATE:
      if (execution_state_before == ExecutionState::HALTED)
      {
        close_execution_cb(ExecutionResult::TERMINATED_PREMATURELY,
                           termination_reason_.empty() ? "Terminated by control command" : termination_reason_);
        return;
      }

      // Fall through to halt tree before termination
      [[fallthrough]];
    case ControlCommand::HALT:
      // Check if already halted
      if (execution_state_before != ExecutionState::HALTED)
      {
        try
        {
          tree_ptr_->haltTree();
        }
        catch (const std::exception& e)
        {
          close_execution_cb(ExecutionResult::ERROR, "Error during haltTree() on command " + toStr(control_command_) +
                                                         ": " + std::string(e.what()));
        }
      }
      return;
    default:
      throw std::logic_error("Handling of control command " + std::to_string(static_cast<int>(control_command_)) +
                             " '" + toStr(control_command_) + "' is not implemented.");
  }

  /* Tick the tree instance */

  state_observer_ptr_->setLogging(executor_params_.state_change_logger);
  BT::NodeStatus bt_status = BT::NodeStatus::IDLE;
  try
  {
    // It is important to tick EXACTLY once to prevent loops induced by BT nodes from blocking
    bt_status = tree_ptr_->tickExactlyOnce();
  }
  catch (const std::exception& e)
  {
    std::string msg = "Ran into an exception during tick: " + std::string(e.what());
    try
    {
      tree_ptr_->haltTree();  // Try to halt tree before aborting
    }
    catch (const std::exception& e)
    {
      msg += "\nDuring haltTree(), another exception occured: " + std::string(e.what());
    }
    close_execution_cb(ExecutionResult::ERROR, msg);
    return;
  }

  /* Continue execution until tree is not running anymore */

  if (bt_status == BT::NodeStatus::RUNNING)
    return;

  /* Determine how to handle the behavior tree execution result */

  if (!(bt_status == BT::NodeStatus::SUCCESS || bt_status == BT::NodeStatus::FAILURE))
  {
    throw std::logic_error("bt_status is " + BT::toStr(bt_status) +
                           ". Must be one of SUCCESS or FAILURE at this point.");
  }
  const bool success = bt_status == BT::NodeStatus::SUCCESS;
  switch (onTreeExit(success))
  {
    case TreeExitBehavior::CLOSE:
      close_execution_cb(success ? ExecutionResult::TREE_SUCCEEDED : ExecutionResult::TREE_FAILED,
                         "Closed on tree result " + BT::toStr(bt_status));
      return;
    case TreeExitBehavior::RESTART:
      control_command_ = ControlCommand::RUN;
      return;
  }

  throw std::logic_error("Execution routine is not intended to proceed to this statement.");
}

bool TreeExecutorBase::onInitialTick()
{
  return true;
}

bool TreeExecutorBase::onTick()
{
  return true;
}

TreeExecutorBase::TreeExitBehavior TreeExecutorBase::onTreeExit(bool /*success*/)
{
  return TreeExitBehavior::CLOSE;
}

void TreeExecutorBase::onClose(const ExecutionResult& /*result*/)
{
}

void TreeExecutorBase::setControlCommand(ControlCommand cmd)
{
  control_command_ = cmd;
}

void TreeExecutorBase::setExecutorParameters(const ExecutorParams& p)
{
  executor_params_ = p;
}

rclcpp::Node::SharedPtr TreeExecutorBase::getNodePtr()
{
  return node_ptr_;
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr TreeExecutorBase::get_node_base_interface()
{
  return node_ptr_->get_node_base_interface();
}

TreeBlackboardSharedPtr TreeExecutorBase::getGlobalBlackboardPtr()
{
  return global_blackboard_ptr_;
}

TreeExecutorBase::ExecutorParams TreeExecutorBase::getExecutorParameters()
{
  return executor_params_;
}

BTStateObserver& TreeExecutorBase::getStateObserver()
{
  return *state_observer_ptr_;
}

std::string toStr(TreeExecutorBase::ExecutionState state)
{
  switch (state)
  {
    case TreeExecutorBase::ExecutionState::IDLE:
      return "IDLE";
    case TreeExecutorBase::ExecutionState::STARTING:
      return "IDLE";
    case TreeExecutorBase::ExecutionState::RUNNING:
      return "RUNNING";
    case TreeExecutorBase::ExecutionState::PAUSED:
      return "PAUSED";
    case TreeExecutorBase::ExecutionState::HALTED:
      return "HALTED";
  }
  return "undefined";
}

std::string toStr(TreeExecutorBase::ControlCommand cmd)
{
  switch (cmd)
  {
    case TreeExecutorBase::ControlCommand::RUN:
      return "RUN";
    case TreeExecutorBase::ControlCommand::PAUSE:
      return "PAUSE";
    case TreeExecutorBase::ControlCommand::HALT:
      return "HALT";
    case TreeExecutorBase::ControlCommand::TERMINATE:
      return "TERMINATE";
  }
  return "undefined";
}

std::string toStr(TreeExecutorBase::TreeExitBehavior behavior)
{
  switch (behavior)
  {
    case TreeExecutorBase::TreeExitBehavior::CLOSE:
      return "CLOSE";
    case TreeExecutorBase::TreeExitBehavior::RESTART:
      return "RESTART";
  }
  return "undefined";
}

std::string toStr(TreeExecutorBase::ExecutionResult result)
{
  switch (result)
  {
    case TreeExecutorBase::ExecutionResult::TREE_SUCCEEDED:
      return "TREE_SUCCEEDED";
    case TreeExecutorBase::ExecutionResult::TREE_FAILED:
      return "TREE_FAILED";
    case TreeExecutorBase::ExecutionResult::TERMINATED_PREMATURELY:
      return "TERMINATED_PREMATURELY";
    case TreeExecutorBase::ExecutionResult::START_REJECTED:
      return "START_REJECTED";
    case TreeExecutorBase::ExecutionResult::ERROR:
      return "ERROR";
  }
  return "undefined";
}

}  // namespace auto_apms_behavior_tree
