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

#include "auto_apms_behavior_tree/executor/executor_node.hpp"

namespace auto_apms_behavior_tree
{

class NoUndeclaredParamsExecutorNode : public TreeExecutorNode
{
public:
  NoUndeclaredParamsExecutorNode(rclcpp::NodeOptions options)
  : TreeExecutorNode(
      _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME, TreeExecutorNodeOptions(options)
                                                         .enableScriptingEnumParameters(false, false)
                                                         .enableGlobalBlackboardParameters(false, false))
  {
  }
};

class OnlyScriptingEnumParamsExecutorNode : public TreeExecutorNode
{
public:
  OnlyScriptingEnumParamsExecutorNode(rclcpp::NodeOptions options)
  : TreeExecutorNode(
      _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME, TreeExecutorNodeOptions(options)
                                                         .enableScriptingEnumParameters(true, true)
                                                         .enableGlobalBlackboardParameters(false, false))
  {
  }
};

class OnlyBlackboardParamsExecutorNode : public TreeExecutorNode
{
public:
  OnlyBlackboardParamsExecutorNode(rclcpp::NodeOptions options)
  : TreeExecutorNode(
      _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME, TreeExecutorNodeOptions(options)
                                                         .enableScriptingEnumParameters(false, false)
                                                         .enableGlobalBlackboardParameters(true, true))
  {
  }
};

class OnlyInitialScriptingEnumParamsExecutorNode : public TreeExecutorNode
{
public:
  OnlyInitialScriptingEnumParamsExecutorNode(rclcpp::NodeOptions options)
  : TreeExecutorNode(
      _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME, TreeExecutorNodeOptions(options)
                                                         .enableScriptingEnumParameters(true, false)
                                                         .enableGlobalBlackboardParameters(false, false))
  {
  }
};

class OnlyInitialBlackboardParamsExecutorNode : public TreeExecutorNode
{
public:
  OnlyInitialBlackboardParamsExecutorNode(rclcpp::NodeOptions options)
  : TreeExecutorNode(
      _AUTO_APMS_BEHAVIOR_TREE__EXECUTOR_DEFAULT_NAME, TreeExecutorNodeOptions(options)
                                                         .enableScriptingEnumParameters(false, false)
                                                         .enableGlobalBlackboardParameters(true, false))
  {
  }
};

}  // namespace auto_apms_behavior_tree

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_tree::TreeExecutorNode)
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_tree::NoUndeclaredParamsExecutorNode)
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_tree::OnlyScriptingEnumParamsExecutorNode)
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_tree::OnlyBlackboardParamsExecutorNode)
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_tree::OnlyInitialScriptingEnumParamsExecutorNode)
RCLCPP_COMPONENTS_REGISTER_NODE(auto_apms_behavior_tree::OnlyInitialBlackboardParamsExecutorNode)