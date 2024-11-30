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

#include <chrono>
#include <functional>
#include <future>

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_behavior_tree/executor/state_observer.hpp"
#include "auto_apms_behavior_tree_core/definitions.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/node.hpp"

namespace auto_apms_behavior_tree
{

class TreeExecutorBase
{
public:
  enum class ExecutionState : uint8_t
  {
    IDLE,
    STARTING,
    RUNNING,
    PAUSED,
    HALTED
  };
  enum class ControlCommand : uint8_t
  {
    RUN,
    PAUSE,
    HALT,
    TERMINATE
  };
  enum class TreeExitBehavior : uint8_t
  {
    TERMINATE,
    RESTART
  };
  enum class ExecutionResult : uint8_t
  {
    TREE_SUCCEEDED,
    TREE_FAILED,
    TERMINATED_PREMATURELY,
    ERROR
  };

private:
  using TerminationCallback = std::function<void(ExecutionResult, const std::string &)>;

public:
  TreeExecutorBase(
    rclcpp::Node::SharedPtr node_ptr, rclcpp::CallbackGroup::SharedPtr tree_node_callback_group_ptr = nullptr);

  std::shared_future<ExecutionResult> startExecution(
    TreeConstructor make_tree, double tick_rate_sec = 0.1, int groot2_port = -1);

  template <typename TimeRepT = int64_t, typename TimeT = std::milli>
  std::shared_future<ExecutionResult> startExecution(
    TreeConstructor make_tree, const std::chrono::duration<TimeRepT, TimeT> & tick_rate, int groot2_port = -1);

private:
  void tick_callback_(TerminationCallback termination_callback);

  /* Virtual methods */

  virtual bool onInitialTick();

  virtual bool onTick();

  virtual bool afterTick();

  virtual TreeExitBehavior onTreeExit(bool success);

  virtual void onTermination(const ExecutionResult & result);

public:
  void setControlCommand(ControlCommand cmd);

  bool isBusy();

  ExecutionState getExecutionState();

  std::string getTreeName();

  /**
   * @brief Get a shared pointer to the global blackboard instance.
   *
   * The global blackboard is used as a root blackboard on every tree that is being created. This means, that all
   * entries of the global blackboard are publicly available to the trees at runtime and can be queried using the '@'
   * prefix.
   *
   * \note This method creates a copy of the internal shared pointer object, so you are not able to modify the internal
   * pointer variable itself.
   */
  TreeBlackboardSharedPtr getGlobalBlackboardPtr();

  void clearGlobalBlackboard();

  TreeStateObserver & getStateObserver();

  /// Get the node's base interface. Is required to be able to register derived classes as ROS2 components.
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

  rclcpp::CallbackGroup::SharedPtr getTreeNodeWaitablesCallbackGroupPtr();

  rclcpp::executors::SingleThreadedExecutor::SharedPtr getTreeNodeWaitablesExecutorPtr();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  const rclcpp::Logger logger_;

private:
  rclcpp::CallbackGroup::SharedPtr tree_node_waitables_callback_group_ptr_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr tree_node_waitables_executor_ptr_;
  TreeBlackboardSharedPtr global_blackboard_ptr_;
  std::unique_ptr<Tree> tree_ptr_;
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_ptr_;
  std::unique_ptr<TreeStateObserver> state_observer_ptr_;
  rclcpp::TimerBase::SharedPtr execution_timer_ptr_;
  ExecutionState prev_execution_state_;
  ControlCommand control_command_;
  bool execution_stopped_;
  std::string termination_reason_;
};

std::string toStr(TreeExecutorBase::ExecutionState state);

std::string toStr(TreeExecutorBase::ControlCommand cmd);

std::string toStr(TreeExecutorBase::TreeExitBehavior behavior);

std::string toStr(TreeExecutorBase::ExecutionResult result);

// #####################################################################################################################
// ################################              DEFINITIONS              ##############################################
// #####################################################################################################################

template <typename TimeRepT, typename TimeT>
inline std::shared_future<TreeExecutorBase::ExecutionResult> TreeExecutorBase::startExecution(
  TreeConstructor make_tree, const std::chrono::duration<TimeRepT, TimeT> & tick_rate, int groot2_port)
{
  return startExecution(
    make_tree, std::chrono::duration_cast<std::chrono::duration<double>>(tick_rate).count(), groot2_port);
}

}  // namespace auto_apms_behavior_tree
