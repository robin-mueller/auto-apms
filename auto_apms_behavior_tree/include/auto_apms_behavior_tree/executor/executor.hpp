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

#include <functional>
#include <future>

#include "auto_apms_behavior_tree/definitions.hpp"
#include "auto_apms_behavior_tree/executor/state_observer.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "executor_params.hpp"
#include "rclcpp/node.hpp"

namespace auto_apms_behavior_tree
{

class TreeExecutor
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
  using CreateTreeCallback = std::function<Tree(TreeBlackboardSharedPtr)>;
  using ExecutorParams = executor_params::Params;

  TreeExecutor(rclcpp::Node::SharedPtr node_ptr);

  std::shared_future<ExecutionResult> startExecution(CreateTreeCallback create_tree_cb);

  bool isBusy();

  ExecutionState getExecutionState();

  std::string getTreeName();

private:
  void execution_routine_(TerminationCallback termination_callback);

  /* Virtual member functions */

  virtual bool onInitialTick();

  virtual bool onTick();

  virtual TreeExitBehavior onTreeExit(bool success);

  virtual void onTermination(const ExecutionResult & result);

public:
  /* Setter functions */

  void setControlCommand(ControlCommand cmd);

  void setExecutorParameters(const ExecutorParams & p);

  /* Getter functions */

  /// Get a pointer to the internal ROS2 node instance.
  rclcpp::Node::SharedPtr getNodePtr();

  /// Get the node's base interface. Is required to be able to register derived classes as ROS2 components.
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

  /**
   * @brief Get a pointer to the global blackboard instance.
   *
   * The global blackboard is used as a root blackboard on every tree that is being created. This means, that all
   * entries of the global blackboard are publicly available to the trees at runtime.
   */
  TreeBlackboardSharedPtr getGlobalBlackboardPtr();

  ExecutorParams getExecutorParameters();

  BTStateObserver & getStateObserver();

private:
  rclcpp::Node::SharedPtr node_ptr_;
  ExecutorParams executor_params_;
  TreeBlackboardSharedPtr global_blackboard_ptr_;
  std::unique_ptr<Tree> tree_ptr_;
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_ptr_;
  std::unique_ptr<BTStateObserver> state_observer_ptr_;
  rclcpp::TimerBase::SharedPtr execution_timer_ptr_;
  ExecutionState prev_execution_state_;
  ControlCommand control_command_;
  bool execution_stopped_;
  std::string termination_reason_;
};

std::string toStr(TreeExecutor::ExecutionState state);

std::string toStr(TreeExecutor::ControlCommand cmd);

std::string toStr(TreeExecutor::TreeExitBehavior behavior);

std::string toStr(TreeExecutor::ExecutionResult result);

}  // namespace auto_apms_behavior_tree
