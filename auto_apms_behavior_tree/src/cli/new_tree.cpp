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
                 "file.\n\t2.) Optional: Node plugin manifest identities of installed resources used to create the "
                 "node model. Any arguments after the first one are interpreted as resource identities.\n";
    std::cerr << "Usage: new_tree <tree_file_path> [<node_manifest_identity> ...].\n";
    return EXIT_FAILURE;
  }

  const std::filesystem::path tree_file_path = std::filesystem::absolute(argv[1]);
  core::NodeManifest node_manifest;
  for (int i = 2; i < argc; ++i) {
    node_manifest.merge(core::NodeManifest::fromResourceIdentity(argv[i]));
  }

  // Make sure path is not empty
  if (tree_file_path.empty()) {
    throw std::runtime_error("Argument tree_file_path is empty.");
  }

  // Require correct extensions
  if (tree_file_path.extension() != ".xml") {
    throw std::runtime_error("Output file '" + tree_file_path.string() + "' has wrong extension. Must be '.xml'.");
  }

  // Make sure that there is no content inside the file
  if (std::filesystem::exists(tree_file_path)) {
    std::ifstream file(tree_file_path);
    if (!file.is_open()) {
      throw std::runtime_error("Couldn't open output file'" + tree_file_path.string() + "' to check if it is empty.");
    }
    // Read the file character by character
    char ch;
    while (file.get(ch)) {
      if (!std::isspace(static_cast<unsigned char>(ch))) {
        // Found a non-whitespace character
        throw std::runtime_error("Output file '" + tree_file_path.string() + "' is not empty.");
      }
    }
  }

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node_ptr = std::make_shared<rclcpp::Node>("_new_tree_temp_node");
  auto_apms_util::exposeToDebugLogging(node_ptr->get_logger());

  // Prepare template document
  core::TreeBuilder builder(node_ptr);
  builder.loadNodePlugins(node_manifest);
  core::TreeBuilder::TreeElement tree = builder.newTree(NEW_TREE_NAME).makeRoot();

  // Insert template children
  tree.insertNode("AlwaysSuccess");

  // Add node model
  if (!node_manifest.getInternalMap().empty()) {
    builder.addNodeModelToDocument(false);
  }

  // Write tree
  std::ofstream out_stream(tree_file_path);
  if (out_stream.is_open()) {
    out_stream << builder.writeTreeDocumentToString();
    out_stream.close();
  } else {
    throw std::runtime_error("Error opening behavior tree output file '" + tree_file_path.string() + "'.");
  }

  std::cout << "Wrote behavior tree to file " << tree_file_path << std::endl;

  return EXIT_SUCCESS;
}
