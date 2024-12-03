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

#include <map>

#include "auto_apms_behavior_tree_core/node/node_registration_options.hpp"
#include "behaviortree_cpp/basic_types.h"

namespace auto_apms_behavior_tree::core
{

class NodeModelType
{
public:
  using PortValues = std::map<std::string, std::string>;
  using PortInfos = std::map<std::string, BT::PortInfo>;

  NodeModelType(const std::string & type, const std::string & registration_options = "");

  virtual ~NodeModelType() = default;

  virtual std::string getRegistrationName() const = 0;

  const BT::NodeType & getNodeType() const;

  const NodeRegistrationOptions & getRegistrationOptions() const;

  std::vector<std::string> getPortNames() const;

  const PortValues & getPortValues() const;

  const PortInfos & getPortInfos() const;

protected:
  const BT::NodeType node_type_;
  const NodeRegistrationOptions registration_options_;
  PortValues port_values_;
  PortInfos port_infos_;
};

}  // namespace auto_apms_behavior_tree::core