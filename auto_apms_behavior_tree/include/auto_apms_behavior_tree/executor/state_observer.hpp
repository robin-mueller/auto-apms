// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>

#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree
{

/**
 * @brief State observer for a particular behavior tree object that writes introspection and debugging information
 * related to behavior tree node transitions to a ROS 2 logger.
 *
 * This is integrated with TreeExecutorNode.
 */
class TreeStateObserver : public BT::StatusChangeLogger
{
public:
  /**
   * @brief Constructor.
   * @param tree Behavior tree object to attach to.
   * @param node_logger ROS 2 logger that this observer writes state transitions of behavior tree nodes to.
   * @param max_logging_interval Maximum logging interval. `0` means to log every state transition regardless the
   * interval between them.
   */
  TreeStateObserver(
    const BT::Tree & tree, const rclcpp::Logger & node_logger,
    std::chrono::seconds max_logging_interval = std::chrono::seconds(0));

  /**
   * @brief Reset the internal state variables.
   */
  virtual void flush() override;

  /**
   * @brief Configure whether the observer should write to the logger.
   * @param active `true` to allow to write to the logger, `false` to disable it.
   */
  void setLogging(bool active);

  /**
   * @brief Get all names of action nodes that returned `BT::NodeStatus::RUNNING` since the last time
   * `TreeStateObserver::flush` was called.
   * @return Names of action nodes that were running recently.
   */
  const std::vector<std::string> & getRunningActionHistory() const;

  /**
   * @brief Get the name of the last action node that returned `BT::NodeStatus::RUNNING`.
   * @return Name of the action node that was running most recently.
   */
  const std::string & getLastRunningActionName() const;

private:
  // Creates a bitmask that uniquely identifies a node's state change
  uint16_t createStateChangeBitmask(BT::NodeStatus prev_status, BT::NodeStatus curr_status);

  virtual void callback(
    BT::Duration timestamp, const BT::TreeNode & node, BT::NodeStatus prev_status, BT::NodeStatus status) override;

  const rclcpp::Logger logger_;
  const std::string root_tree_id_;
  const std::chrono::seconds max_logging_interval_;
  bool logging_active_{false};
  std::vector<std::string> running_action_history_;
  std::string last_running_action_name_;
  std::map<std::pair<uint16_t, uint16_t>, BT::Duration> last_log_map_;
};

}  // namespace auto_apms_behavior_tree
