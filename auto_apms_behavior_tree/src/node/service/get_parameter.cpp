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

#include "auto_apms_behavior_tree/util/conversion.hpp"
#include "auto_apms_behavior_tree_core/node.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/parameter_value.hpp"

#define INPUT_KEY_PARAM_NAME "parameter"
#define OUTPUT_KEY_PARAM_VALUE "value"
#define INPUT_KEY_NODE_NAME "node"

namespace auto_apms_behavior_tree
{

template <typename T>
class GetParameterTemplate : public core::RosServiceNode<rcl_interfaces::srv::GetParameters>
{
public:
  GetParameterTemplate(const std::string & instance_name, const Config & config, const Context & context)
  : RosServiceNode(instance_name, config, context)
  {
    if (
      config.input_ports.find(INPUT_KEY_NODE_NAME) == config.input_ports.end() ||
      config.input_ports.at(INPUT_KEY_NODE_NAME).empty()) {
      // Refer to this ROS 2 node as the target if respective input port is empty
      createClient(context_.getFullyQualifiedROSNodeName() + "/get_parameters");
    }
  }

  static BT::PortsList providedPorts()
  {
    // We do not use the default port for the service name
    return {
      BT::InputPort<std::string>(INPUT_KEY_NODE_NAME, "Name of the targeted ROS 2 node."),
      BT::OutputPort<T>(OUTPUT_KEY_PARAM_VALUE, "Output port for the parameter's value."),
      BT::InputPort<std::string>(INPUT_KEY_PARAM_NAME, "Name of the parameter to get.")};
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    if (const BT::Expected<std::string> val = getInput<std::string>(INPUT_KEY_PARAM_NAME);
        val && !val.value().empty()) {
      request->names.push_back(val.value());
      return true;
    } else {
      RCLCPP_WARN(
        context_.getLogger(), "%s - Parameter name must not be empty.",
        context_.getFullyQualifiedTreeNodeName(this).c_str());
      return false;
    }
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    if (response->values.empty()) {
      throw exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) + " - Response vector doesn't contain any values");
    }
    rclcpp::ParameterValue val(response->values[0]);
    if constexpr (std::is_same_v<BT::Any, T>) {
      const BT::Expected<BT::Any> expected = createAnyFromParameterValue(val);
      if (!expected) {
        RCLCPP_WARN(
          context_.getLogger(), "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
          expected.error().c_str());
        return BT::NodeStatus::FAILURE;
      }
      if (const BT::Result res = setOutput(OUTPUT_KEY_PARAM_VALUE, expected.value()); !res) {
        RCLCPP_WARN(
          context_.getLogger(), "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), res.error().c_str());
        return BT::NodeStatus::FAILURE;
      }
    } else {
      if (const BT::Result res = setOutput(OUTPUT_KEY_PARAM_VALUE, val.get<T>()); !res) {
        RCLCPP_WARN(
          context_.getLogger(), "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), res.error().c_str());
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::SUCCESS;
  }
};

// Use BT::Any as the type of the blackboard entry set from the parameter, so type may change dynamically until the
// entry is read. Behavior trees trying to read the entry must know the current type and cast correspondingly.
class GetParameter : public GetParameterTemplate<BT::Any>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterBool : public GetParameterTemplate<bool>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterInt : public GetParameterTemplate<int64_t>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterDouble : public GetParameterTemplate<double>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterString : public GetParameterTemplate<std::string>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterByteVec : public GetParameterTemplate<std::vector<uint8_t>>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterBoolVec : public GetParameterTemplate<std::vector<bool>>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterIntVec : public GetParameterTemplate<std::vector<int64_t>>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterDoubleVec : public GetParameterTemplate<std::vector<double>>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

class GetParameterStringVec : public GetParameterTemplate<std::vector<std::string>>
{
public:
  using GetParameterTemplate::GetParameterTemplate;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameter)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterBool)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterInt)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterDouble)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterString)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterByteVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterBoolVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterIntVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterDoubleVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::GetParameterStringVec)