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
#include "auto_apms_behavior_tree_core/node/node_registration_interface.hpp"
#include "auto_apms_util/logging.hpp"
#include "auto_apms_util/string.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "create_node_model: Missing inputs! The program requires: \n\t1.) The path to the node plugin "
                 "manifest.\n\t2. The exhaustive list of libraries to be loaded by ClassLoader (Seperated by "
                 "';').\n\t3.) The xml file to "
                 "store the model.\n";
    std::cerr << "Usage: create_node_model <manifest_file> <library_paths> <output_file>.\n";
    return EXIT_FAILURE;
  }

  try {
    const std::string manifest_file = std::filesystem::absolute(argv[1]).string();
    const std::vector<std::string> library_paths = auto_apms_util::splitString(argv[2], ";", false);
    const std::filesystem::path output_file = std::filesystem::absolute(argv[3]);

    // Ensure that arguments are not empty
    if (manifest_file.empty()) {
      throw std::runtime_error("Argument manifest_file must not be empty.");
    }
    if (output_file.empty()) {
      throw std::runtime_error("Argument output_file must not be empty.");
    }

    // Ensure correct extensions
    if (output_file.extension() != ".xml") {
      throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.xml'.");
    }

    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<rclcpp::Node>("_generate_node_model_temp_node");
    auto_apms_util::exposeToDebugLogging(node_ptr->get_logger());

    BT::BehaviorTreeFactory factory;
    const auto manifest = core::NodeManifest::fromFile(manifest_file);

    /**
     * NOTE: We have to use the low level class loader here because the pluginlib::ClassLoader API doesn't allow
     * customizing the internal node/library allocation map.
     */

    // Instatiate loaders for all libraries in library_paths (We don't use class_loader::MultiLibraryClassLoader because
    // we want to keep track of the libraries that the nodes come from for debugging purposes)
    std::vector<std::unique_ptr<class_loader::ClassLoader>> class_loaders;
    for (const auto & path : library_paths) class_loaders.push_back(std::make_unique<class_loader::ClassLoader>(path));

    // Walk manifest and register all plugins with BT::BehaviorTreeFactory
    for (const auto & [node_name, params] : manifest.getInternalMap()) {
      const std::string required_class_name =
        "auto_apms_behavior_tree::core::NodeRegistrationTemplate<" + params.class_name + ">";

      class_loader::ClassLoader * loader = nullptr;
      size_t index = 0;
      do {
        loader = class_loaders.at(index++).get();
      } while (index < class_loaders.size() &&
               !loader->isClassAvailable<core::NodeRegistrationInterface>(required_class_name));

      if (!loader) {
        throw std::runtime_error(
          "Node '" + node_name + " (Class: " + params.class_name +
          ")' cannot be loaded, because the required registration class '" + required_class_name +
          "' couldn't be found. Check that the class name is spelled correctly and "
          "registered "
          "by calling auto_apms_behavior_tree_register_nodes() in the CMakeLists.txt of the "
          "corresponding package. Also make sure that you called the "
          "AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE macro in the source file.");
      }

      RCLCPP_DEBUG(
        node_ptr->get_logger(), "Loading behavior tree node '%s' (Class: %s) from library %s.", node_name.c_str(),
        params.class_name.c_str(), loader->getLibraryPath().c_str());

      try {
        const auto plugin_instance = loader->createUniqueInstance<core::NodeRegistrationInterface>(required_class_name);
        core::RosNodeContext ros_node_context(node_ptr, params);  // Values don't matter when not instantiating it
        plugin_instance->registerWithBehaviorTreeFactory(factory, node_name, &ros_node_context);
      } catch (const std::exception & e) {
        throw std::runtime_error(
          "Failed to load and register node '" + node_name + " (Class: " + params.class_name + ")': " + e.what() + ".");
      }
    }

    // Generate and write node model
    std::ofstream out_stream{output_file};
    if (out_stream.is_open()) {
      out_stream << BT::writeTreeNodesModelXML(factory);
      out_stream.close();
    } else {
      throw std::runtime_error("Error opening node model output file '" + output_file.string() + "'.");
    }
  } catch (const std::exception & e) {
    std::cerr << "ERROR (create_node_model): " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
