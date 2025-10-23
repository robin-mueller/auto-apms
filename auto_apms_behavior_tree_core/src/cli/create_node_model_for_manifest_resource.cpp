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

#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "auto_apms_util/string.hpp"

using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 3) {
    std::cerr << "create_node_model_for_manifest_resource: Wrong number of arguments!\n";
    std::cerr << "Usage: create_node_model_for_manifest_resource <output_file>\n";
    std::cerr << "       create_node_model_for_manifest_resource <output_file> <node_manifest_resource_identity>\n";
    std::cerr << "\nIf only output_file is provided, generates model for native BehaviorTree.CPP nodes.\n";
    std::cerr << "If both arguments are provided, generates model for the specified node manifest resource.\n";
    return EXIT_FAILURE;
  }

  try {
    std::filesystem::path output_file;
    std::string manifest_resource_identity;
    bool use_native_only = false;

    // First argument is always the output file
    output_file = std::filesystem::absolute(auto_apms_util::trimWhitespaces(argv[1]));

    if (argc == 2) {
      // Single argument - just output file, use native nodes only
      use_native_only = true;
    } else {
      // Two arguments - output file and manifest resource identity
      manifest_resource_identity = auto_apms_util::trimWhitespaces(argv[2]);
    }

    // Ensure correct extension
    if (output_file.extension() != ".xml") {
      throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.xml'.");
    }

    // Create output directory if it doesn't exist
    std::filesystem::create_directories(output_file.parent_path());

    // Create TreeDocument for the node model
    core::TreeDocument doc;

    if (use_native_only) {
      // Just use native nodes - no need to register anything
    } else {
      // Register all nodes with the document
      doc.registerNodes(core::NodeManifest::fromResource(manifest_resource_identity));
    }

    // Get the node model and add it to the document, then write to file
    doc.addNodeModel(doc.getNodeModel(use_native_only));
    doc.writeToFile(output_file.string());

  } catch (const std::exception & e) {
    std::cerr << "ERROR (create_node_model_for_manifest_resource): " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
