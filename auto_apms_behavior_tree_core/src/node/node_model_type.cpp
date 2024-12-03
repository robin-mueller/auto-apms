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

#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"

namespace auto_apms_behavior_tree::core
{

NodeModelType::NodeModelType(const std::string & type, const std::string & registration_options)
: node_type_(BT::convertFromString<BT::NodeType>(type)),
  registration_options_(
    registration_options.empty() ? NodeRegistrationOptions() : NodeRegistrationOptions::decode(registration_options))
{
}

const BT::NodeType & NodeModelType::getNodeType() const { return node_type_; }

const NodeRegistrationOptions & NodeModelType::getRegistrationOptions() const { return registration_options_; }

std::vector<std::string> NodeModelType::getPortNames() const
{
  std::vector<std::string> vec;
  vec.reserve(port_infos_.size());
  for (const auto & [name, _] : port_infos_) vec.push_back(name);
  return vec;
}

const NodeModelType::PortValues & NodeModelType::getPortValues() const { return port_values_; }

const NodeModelType::PortInfos & NodeModelType::getPortInfos() const { return port_infos_; }

}  // namespace auto_apms_behavior_tree::core