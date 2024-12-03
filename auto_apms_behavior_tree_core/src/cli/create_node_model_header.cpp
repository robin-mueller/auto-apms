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

#include <tinyxml2.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "behaviortree_cpp/xml_parsing.h"

using namespace auto_apms_behavior_tree;

const std::regex no_typed_setter_getter_types("std::shared_ptr|std::unique_ptr|std::weak_ptr|BT::Any|std::string");

int main(int argc, char ** argv)
{
  bool only_native_nodes = false;
  if (argc == 2) {
    // Only one argument. Is assumed to be the header output file
    only_native_nodes = true;
  } else if (argc < 4) {
    std::cerr
      << "create_node_model_header: Missing inputs! The program requires: \n\t1.) The path to the node plugin "
         "manifest YAML file.\n\t2.) The path to the nodes model XML file.\n\t3.) The path of the output .hpp file.\n";
    std::cerr << "Usage: create_node_model_header <manifest_file> <model_file> <header_path>.\n";
    return EXIT_FAILURE;
  }

  try {
    const std::filesystem::path manifest_file = only_native_nodes ? "unused" : std::filesystem::absolute(argv[1]);
    const std::filesystem::path model_file = only_native_nodes ? "unused" : std::filesystem::absolute(argv[2]);
    const std::filesystem::path header_path =
      only_native_nodes ? std::filesystem::absolute(argv[1]) : std::filesystem::absolute(argv[3]);

    // Ensure that arguments are not empty
    if (manifest_file.empty()) {
      throw std::runtime_error("Argument manifest_file must not be empty.");
    }
    if (model_file.empty()) {
      throw std::runtime_error("Argument model_file must not be empty.");
    }
    if (header_path.empty()) {
      throw std::runtime_error("Argument header_path must not be empty.");
    }

    // Ensure correct extensions
    if (header_path.extension() != ".hpp") {
      throw std::runtime_error("Output file '" + header_path.string() + "' has wrong extension. Must be '.hpp'.");
    }

    core::NodeManifest manifest;
    tinyxml2::XMLDocument model_doc;
    core::TreeBuilder::NodeModelMap model_map;
    if (only_native_nodes) {
      if (
        model_doc.Parse(BT::writeTreeNodesModelXML(BT::BehaviorTreeFactory(), true).c_str()) !=
        tinyxml2::XMLError::XML_SUCCESS) {
        throw std::runtime_error(model_doc.ErrorStr());
      }
      model_map = core::TreeBuilder::getNodeModel(model_doc);
      for (const auto & [name, _] : model_map) {
        core::NodeRegistrationOptions opt;
        opt.class_name = "empty";
        manifest.add(name, opt);
      }
    } else {
      manifest = core::NodeManifest::fromFile(manifest_file);
      if (model_doc.LoadFile(model_file.c_str()) != tinyxml2::XMLError::XML_SUCCESS) {
        throw std::runtime_error(model_doc.ErrorStr());
      }
      model_map = core::TreeBuilder::getNodeModel(model_doc);
    }

    std::ostringstream content;

    // clang-format off
    content << R"(// This header has been generated automatically. DO NOT CHANGE!

#pragma once

#include "behaviortree_cpp/basic_types.h"
#include "auto_apms_behavior_tree_core/convert.hpp"
#include "auto_apms_behavior_tree_core/node/node_model_type.hpp"

namespace auto_apms_behavior_tree::model
{
)";
    // clang-format on
    for (const auto & [node_name, options] : manifest.map()) {
      core::TreeBuilder::NodeModelMap::const_iterator it = model_map.find(node_name);
      if (it == model_map.end()) continue;
      const core::TreeBuilder::NodeModel & model = it->second;
      // clang-format off
      content << R"(
class )" << node_name << R"( : public auto_apms_behavior_tree::core::NodeModelType
{
public:
)" << node_name << R"(() : NodeModelType(")" << model.type << "\"";
      // clang-format on
      if (!only_native_nodes) {
        content << ", R\"(\n" << options.encode() << "\n)\"";
      }
      content << ")\n{\n";
      for (const core::TreeBuilder::NodePortInfo & info : model.port_infos) {
        // clang-format off
        content << "port_infos_.insert(BT::CreatePort<"<< info.port_type <<">(BT::convertFromString<BT::PortDirection>(\"" << info.port_direction << "\"), \"" << info.port_name << "\", \"" << info.port_description << "\"));\n";
        if (!info.port_default.empty() && !BT::TreeNode::isBlackboardPointer(info.port_default)) {
          content << "port_infos_[\"" << info.port_name << "\"].setDefaultValue(BT::convertFromString<" << info.port_type << ">(\"" << info.port_default << "\"));\n";
        }
        // clang-format on
      }
      // clang-format off
      content << R"(}

std::string getRegistrationName() const override final
{
return ")" << node_name << R"(";
}
)";
      // clang-format on
      for (const core::TreeBuilder::NodePortInfo & info : model.port_infos) {
        content << "\n" << node_name << " & set_" << info.port_name << "(const std::string & str";
        if (info.port_default.empty()) {
          content << ")";
        } else {
          content << " = \"" << info.port_default << "\")";
        }
        // clang-format off
        content << R"(
{
port_values_[")" << info.port_name << R"("] = str;
return *this;
}

const std::string & get_)" << info.port_name << R"(_str() const
{
return port_values_.at(")" << info.port_name << R"(");
}
)";
        // clang-format on
        if (std::regex_search(info.port_type, no_typed_setter_getter_types)) continue;
        content << "\n" << node_name << " & set_" << info.port_name << "(const " << info.port_type << " & val)";
        // clang-format off
        content << R"(
{
port_values_[")" << info.port_name << R"("] = BT::toStr(val);
return *this;
}

)" << info.port_type << " get_" << info.port_name << R"(() const
{
return BT::convertFromString<)" << info.port_type << ">(get_" << info.port_name << R"(_str());
}
)";
        // clang-format on
      }
      content << "};\n";
    }
    content << "\n}";

    std::ofstream out_stream(header_path);
    if (out_stream.is_open()) {
      out_stream << content.str();
      out_stream.close();
    } else {
      throw std::runtime_error("Error opening node model header file '" + header_path.string() + "'.");
    }
  } catch (const std::exception & e) {
    std::cerr << "ERROR (create_node_model_header): " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
