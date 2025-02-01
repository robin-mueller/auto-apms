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

#include "auto_apms_behavior_tree/behavior_tree_nodes.hpp"
#include "auto_apms_behavior_tree/executor/executor_base.hpp"
#include "auto_apms_behavior_tree/util/node.hpp"
#include "auto_apms_behavior_tree_core/builder.hpp"
#include "auto_apms_util/filesystem.hpp"
#include "auto_apms_util/string.hpp"
#include "rclcpp/rclcpp.hpp"

#define NEW_TREE_NAME "NewTree"

using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  bool print_help = false;
  if (argc > 1) {
    const std::string arg(argv[1]);
    print_help = "-h" == arg || "--help" == arg;
  }
  if (print_help || argc < 2) {
    std::cerr << "new_tree: The program accepts: \n\t1.) The path of the new behavior tree xml "
                 "file.\n\t2.) Optional: Node manifest identities of installed resources used to create the "
                 "node model. All arguments after the first one are interpreted as resource identities.\n";
    std::cerr << "Usage: new_tree <tree_file_path> [<node_manifest_identity> ...].\n";
    return EXIT_FAILURE;
  }

  const std::filesystem::path tree_file_path = std::filesystem::absolute(auto_apms_util::trimWhitespaces(argv[1]));
  core::NodeManifest node_manifest;
  for (int i = 2; i < argc; ++i) {
    node_manifest.merge(core::NodeManifest::fromResourceIdentity(auto_apms_util::trimWhitespaces(argv[i])));
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
    if (!auto_apms_util::isFileEmpty(tree_file_path.string())) {
      throw std::runtime_error("Output file '" + tree_file_path.string() + "' is not empty.");
    }
  }

  // Prepare document
  core::TreeDocument doc;
  doc.registerNodes(node_manifest);
  core::TreeBuilder::TreeElement tree = doc.newTree(NEW_TREE_NAME).makeRoot();

  // Insert children
  tree.insertNode<model::AlwaysSuccess>();

  // Add node model
  if (!node_manifest.empty()) {
    doc.addNodeModel(false);
  }

  // Write tree
  std::ofstream out_stream(tree_file_path);
  if (out_stream.is_open()) {
    out_stream << doc.writeToString();
    out_stream.close();
  } else {
    throw std::runtime_error("Error opening behavior tree output file '" + tree_file_path.string() + "'.");
  }

  std::cout << "Wrote behavior tree to file " << tree_file_path << std::endl;

  return EXIT_SUCCESS;
}
