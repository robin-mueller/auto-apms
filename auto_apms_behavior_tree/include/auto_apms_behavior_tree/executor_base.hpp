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

#include "auto_apms_behavior_tree/creator.hpp"
#include "auto_apms_behavior_tree/state_observer.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "executor_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree {

class BTExecutorBase
{
   public:
    enum class ExecutionState : uint8_t { IDLE, RUNNING, PAUSED, HALTED, TERMINATED };
    enum class ControlCommand : uint8_t { RUN, PAUSE, HALT, TERMINATE };
    enum class TreeExitBehavior : uint8_t { CLOSE, RESTART };
    enum class ClosureResult : uint8_t { TREE_SUCCEEDED, TREE_FAILED, TERMINATED_EARLY, ERROR };

   private:
    using CloseExecutionCallback = std::function<void(ClosureResult, const std::string&)>;

   public:
    using ExecutorParams = executor_params::Params;
    using NodePluginsParams = node_plugin_manifest_params::Params;

    BTExecutorBase(rclcpp::Node::SharedPtr node_ptr);

    void CreateTree(BTCreator& tree_creator, const std::string& main_tree_id = "");

    std::shared_future<ClosureResult> Start();

    bool IsStarted();

    ExecutionState GetExecutionState();

    bool SetControlCommand(ControlCommand cmd);

   private:
    void Reset();

    void ExecutionRoutine(CloseExecutionCallback close_callback);

    /* Virtual member functions */

   private:
    virtual bool OnFirstTick();

    virtual void OnTick();

    virtual TreeExitBehavior OnTreeExit(bool success);

    virtual void OnClose();

   public:
    /* Getter functions */

    /// Get a pointer to the internal ROS2 node instance.
    rclcpp::Node::SharedPtr node();

    /// Get the node's base interface.
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

    /**
     * @brief Get a pointer to the global blackboard instance.
     *
     * The global blackboard is used as a root blackboard on every tree that is being created. This means, that all
     * entries of the global blackboard are publicly available to the trees at runtime.
     */
    BT::Blackboard::Ptr global_blackboard();

    ExecutorParams executor_parameters();

    NodePluginsParams node_plugins_parameters();

    const BTStateObserver& state_observer();

   private:
    rclcpp::Node::SharedPtr node_ptr_;
    executor_params::ParamListener executor_param_listener_;
    node_plugin_manifest_params::ParamListener nodes_param_listener_;
    BT::Blackboard::Ptr global_blackboard_ptr_;
    std::unique_ptr<BT::Tree> tree_ptr_;
    std::unique_ptr<BT::Groot2Publisher> groot2_publisher_ptr_;
    std::unique_ptr<BTStateObserver> state_observer_ptr_;
    rclcpp::TimerBase::SharedPtr execution_timer_ptr_;
    ControlCommand control_command_;
    bool execution_stopped_;
};

std::string to_string(BTExecutorBase::ExecutionState state);

std::string to_string(BTExecutorBase::ControlCommand cmd);

}  // namespace auto_apms_behavior_tree
