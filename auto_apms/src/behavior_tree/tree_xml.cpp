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

#include "auto_apms/behavior_tree/tree_xml.hpp"

namespace auto_apms {

BehaviorTreeXML::BehaviorTreeXML(const std::string& tree_file_path) { doc_.LoadFile(tree_file_path.c_str()); }

BehaviorTreeXML::BehaviorTreeXML(const BehaviorTreeResource& tree_resource) : BehaviorTreeXML{tree_resource.tree_path}
{}

std::string BehaviorTreeXML::GetMainID()
{
    if (const auto main_tree_id = doc_.RootElement()->Attribute(MAIN_TREE_ATTRIBUTE_NAME)) return main_tree_id;
    return "";
}

void BehaviorTreeXML::SetMainID(const std::string& main_tree_id)
{
    doc_.RootElement()->SetAttribute(MAIN_TREE_ATTRIBUTE_NAME, main_tree_id.c_str());
}

std::string BehaviorTreeXML::WriteToString()
{
    tinyxml2::XMLPrinter printer;
    doc_.Print(&printer);
    return printer.CStr();
}

}  // namespace auto_apms