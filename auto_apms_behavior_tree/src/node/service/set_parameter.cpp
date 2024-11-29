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
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

#define INPUT_KEY_PARAM_NAME "parameter"
#define INPUT_KEY_PARAM_VALUE "value"
#define INPUT_KEY_NODE_NAME "node"

namespace auto_apms_behavior_tree
{

template <rclcpp::ParameterType param_type>
class SetParameterTemplate : public core::RosServiceNode<rcl_interfaces::srv::SetParameters>
{
public:
  SetParameterTemplate(const std::string & instance_name, const Config & config, const Context & context)
  : RosServiceNode(instance_name, config, context)
  {
    if (
      config.input_ports.find(INPUT_KEY_NODE_NAME) == config.input_ports.end() ||
      config.input_ports.at(INPUT_KEY_NODE_NAME).empty()) {
      // Refer to this ROS 2 node as the target if respective input port is empty
      createClient(context_.getFullyQualifiedROSNodeName() + "/set_parameters");
    }
  }

  static BT::PortsList providedPorts()
  {
    // We do not use the default port for the service name
    return {
      BT::InputPort<std::string>(INPUT_KEY_NODE_NAME, "Name of the targeted ROS 2 node."),
      BT::InputPort<BT::Any>(INPUT_KEY_PARAM_VALUE, "Value of the parameter to be set."),
      BT::InputPort<std::string>(INPUT_KEY_PARAM_NAME, "Name of the parameter to be set."),
    };
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    rcl_interfaces::msg::Parameter parameter;
    if (const BT::Expected<std::string> val = getInput<std::string>(INPUT_KEY_PARAM_NAME);
        val && !val.value().empty()) {
      parameter.name = val.value();
    } else {
      RCLCPP_WARN(
        context_.getLogger(), "%s - Parameter name must not be empty.",
        context_.getFullyQualifiedTreeNodeName(this).c_str());
      return false;
    }

    rcl_interfaces::msg::ParameterValue param_val;
    if (const BT::Expected<BT::Any> any = getInput<BT::Any>(INPUT_KEY_PARAM_VALUE)) {
      const BT::Expected<rclcpp::ParameterValue> expected = createParameterValueFromAny(any.value(), param_type);
      if (!expected) {
        RCLCPP_WARN(
          context_.getLogger(), "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
          expected.error().c_str());
        return false;
      }
      param_val = expected.value().to_value_msg();
    } else {
      // Use default value defined by rcl_interfaces::msg::Parameter if input is not specified
      param_val.type = param_type;
    }

    parameter.value = param_val;
    request->parameters.push_back(parameter);
    return true;
  }
};

// Automatically infer the parameter type from BT::Any
class SetParameter : public SetParameterTemplate<rclcpp::ParameterType::PARAMETER_NOT_SET>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterBool : public SetParameterTemplate<rclcpp::ParameterType::PARAMETER_BOOL>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterInt : public SetParameterTemplate<rclcpp::ParameterType::PARAMETER_INTEGER>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterDouble : public SetParameterTemplate<rclcpp::ParameterType::PARAMETER_DOUBLE>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterString : public SetParameterTemplate<rclcpp::ParameterType::PARAMETER_STRING>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterByteVec : public SetParameterTemplate<rclcpp::ParameterType::PARAMETER_BYTE_ARRAY>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterBoolVec : public SetParameterTemplate<rclcpp::ParameterType::PARAMETER_BOOL_ARRAY>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterIntVec : public SetParameterTemplate<rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterDoubleVec : public SetParameterTemplate<rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

class SetParameterStringVec : public SetParameterTemplate<rclcpp::ParameterType::PARAMETER_STRING_ARRAY>
{
public:
  using SetParameterTemplate::SetParameterTemplate;
};

}  // namespace auto_apms_behavior_tree

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameter)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterBool)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterInt)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterDouble)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterString)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterByteVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterBoolVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterIntVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterDoubleVec)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_behavior_tree::SetParameterStringVec)