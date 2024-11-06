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

#include "auto_apms_behavior_tree/executor/state_observer.hpp"

#include <chrono>

namespace auto_apms_behavior_tree
{

BTStateObserver::BTStateObserver(
  const BT::Tree & tree, const rclcpp::Logger & node_logger, std::chrono::seconds max_logging_rate)
: StatusChangeLogger{tree.rootNode()},
  logger_{node_logger},
  root_tree_id_{tree.subtrees[0]->tree_ID},
  max_logging_rate_{max_logging_rate}
{
}

void BTStateObserver::flush() { running_action_history_.clear(); }

void BTStateObserver::setLogging(bool active) { logging_active_ = active; }

const std::vector<std::string> & BTStateObserver::getRunningActionHistory() const { return running_action_history_; }

const std::string & BTStateObserver::getLastRunningActionName() const { return last_running_action_name_; }

uint16_t BTStateObserver::createStateChangeBitmask(BT::NodeStatus prev_status, BT::NodeStatus curr_status)
{
  return static_cast<uint16_t>(prev_status) << 8 | static_cast<uint16_t>(curr_status);
}

void BTStateObserver::callback(
  BT::Duration timestamp, const BT::TreeNode & node, BT::NodeStatus prev_status, BT::NodeStatus status)
{
  if (node.type() == BT::NodeType::ACTION && status == BT::NodeStatus::RUNNING) {
    auto name = node.registrationName() == node.name() ? node.name() : node.registrationName() + ": " + node.name();
    running_action_history_.push_back(name);
    last_running_action_name_ = name;
  }

  /**
   * Write to ROS2 logger but respect a maximum interval if a specific node triggers the same state
   * transitions (e.g. conditions in reactive control statements or loops).
   */
  if (!logging_active_) return;
  const auto key = std::make_pair(node.UID(), createStateChangeBitmask(prev_status, status));
  const bool is_first_log = last_log_map_.count(key) == 0;

  if (is_first_log || timestamp - last_log_map_[key] > max_logging_rate_) {
    if (node.registrationName() == node.name()) {
      RCLCPP_INFO(
        logger_, "[%s] %s '%s' -- %s -> %s", root_tree_id_.c_str(), BT::toStr(node.type()).c_str(), node.name().c_str(),
        BT::toStr(prev_status, true).c_str(), BT::toStr(status, true).c_str());
    } else {
      RCLCPP_INFO(
        logger_, "[%s] %s '%s: %s' -- %s -> %s", root_tree_id_.c_str(), BT::toStr(node.type()).c_str(),
        node.registrationName().c_str(), node.name().c_str(), BT::toStr(prev_status, true).c_str(),
        BT::toStr(status, true).c_str());
    }
    last_log_map_[key] = timestamp;
  }
}

}  // namespace auto_apms_behavior_tree
