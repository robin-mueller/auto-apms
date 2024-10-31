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

#include "behaviortree_cpp/loggers/abstract_logger.h"
#include "rclcpp/rclcpp.hpp"

namespace auto_apms_behavior_tree
{

class BTStateObserver : public BT::StatusChangeLogger
{
public:
  BTStateObserver(const BT::Tree& tree, const rclcpp::Logger& node_logger,
                  std::chrono::seconds max_logging_rate = std::chrono::seconds(0));

  virtual void flush() override;

  void setLogging(bool active);

  const std::vector<std::string>& getRunningActionHistory() const;
  const std::string& getLastRunningActionName() const;

private:
  // Creates a bitmask that uniquely identifies a node's state change
  uint16_t createStateChangeBitmask(BT::NodeStatus prev_status, BT::NodeStatus curr_status);

  virtual void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                        BT::NodeStatus status) override;

  const rclcpp::Logger logger_;
  const std::string root_tree_id_;
  const std::chrono::seconds max_logging_rate_;
  bool logging_active_{ false };
  std::vector<std::string> running_action_history_;
  std::string last_running_action_name_;
  std::map<std::pair<uint16_t, uint16_t>, BT::Duration> last_log_map_;
};

}  // namespace auto_apms_behavior_tree
