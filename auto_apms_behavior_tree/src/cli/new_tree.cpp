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

#include <filesystem>
#include <fstream>
#include <iostream>

#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_util/logging.hpp"
#include "rclcpp/rclcpp.hpp"

#define NEW_TREE_NAME "NewTree"

using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "new_tree: Missing inputs! The program requires: \n\t1.) The path of the new behavior tree xml "
                 "file.\n\t2.) Optional: Node plugin manifest files used to create the node model. Any arguments after "
                 "the first one are interpreted as node plugin manifest paths.\n";
    std::cerr << "Usage: new_tree <tree_file_path> [<node_manifest_file_path> ...].\n";
    return EXIT_FAILURE;
  }

  const std::filesystem::path tree_file_path = std::filesystem::absolute(argv[1]);
  std::vector<std::string> node_manifest_file_paths;
  for (int i = 2; i < argc; ++i) {
    const std::filesystem::path path = std::filesystem::absolute(argv[i]);
    if (!std::filesystem::exists(path)) {
      throw std::runtime_error("Node plugin manifest path '" + path.string() + "' doesn't exist.");
    }
    node_manifest_file_paths.push_back(path);
  }

  // Ensure correct extensions
  if (tree_file_path.extension() != ".xml") {
    throw std::runtime_error("Output file '" + tree_file_path.string() + "' has wrong extension. Must be '.xml'.");
  }

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node_ptr = std::make_shared<rclcpp::Node>("_new_tree_temp_node");
  auto_apms_util::exposeToDebugLogging(node_ptr->get_logger());

  // Prepare document
  core::TreeBuilder builder(node_ptr);
  core::NodeManifest node_manifest = core::NodeManifest::fromFiles(node_manifest_file_paths);
  builder.loadNodePlugins(node_manifest);
  core::TreeBuilder::ElementPtr tree_ele = builder.insertNewTreeElement(NEW_TREE_NAME);
  builder.setRootTreeName(NEW_TREE_NAME);
  builder.insertNewNodeElement(tree_ele, "AlwaysSuccess");

  // Get node model
  if (!node_manifest.getInternalMap().empty()) {
    core::TreeBuilder::DocumentSharedPtr builder_doc_ptr = builder.getDocumentPtr();
    core::TreeBuilder::DocumentSharedPtr model_doc_ptr = builder.getNodeModel(false);  // Structure is already verified
    core::TreeBuilder::ElementPtr model_child =
      model_doc_ptr->RootElement()->FirstChildElement(core::TreeBuilder::TREE_NODE_MODEL_ELEMENT_NAME);

    // Clone the memory of the node model element to the builder document
    tinyxml2::XMLNode * copied_child = model_child->DeepClone(builder_doc_ptr.get());

    // Append the copied child to the root of the builder document
    builder_doc_ptr->RootElement()->InsertEndChild(copied_child);
  }

  // Write tree
  std::ofstream out_stream(tree_file_path);
  if (out_stream.is_open()) {
    out_stream << builder.writeTreeDocumentToString();
    out_stream.close();
  } else {
    throw std::runtime_error("Error opening behavior tree output file '" + tree_file_path.string() + "'.");
  }
  return EXIT_SUCCESS;
}
