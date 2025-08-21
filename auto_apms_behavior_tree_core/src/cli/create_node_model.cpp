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

using namespace auto_apms_behavior_tree;

int main(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "create_node_model: Missing inputs! The program requires: \n\t1.) The path to the node plugin "
                 "manifest.\n\t2. The exhaustive list of libraries to be loaded by ClassLoader (Separated by "
                 "';').\n\t3.) The xml file to store the model.\n";
    std::cerr << "Usage: create_node_model <manifest_file> <library_paths> <output_file>.\n";
    return EXIT_FAILURE;
  }

  try {
    const std::filesystem::path manifest_file = std::filesystem::absolute(auto_apms_util::trimWhitespaces(argv[1]));
    const std::vector<std::string> library_paths = auto_apms_util::splitString(argv[2], ";");
    const std::filesystem::path output_file = std::filesystem::absolute(auto_apms_util::trimWhitespaces(argv[3]));

    if (!std::filesystem::exists(manifest_file)) {
      throw std::runtime_error("File manifest_file must exist.");
    }
    if (library_paths.empty()) {
      throw std::runtime_error("Argument library_paths must not be empty.");
    }
    if (!output_file.has_filename()) {
      throw std::runtime_error("Output file path must include a filename.");
    }

    // Ensure correct extensions
    if (output_file.extension() != ".xml") {
      throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.xml'.");
    }

    const rclcpp::Logger logger = rclcpp::get_logger("create_node_model__" + output_file.stem().string());

    BT::BehaviorTreeFactory factory;
    const auto manifest = core::NodeManifest::fromFile(manifest_file.string());

    /**
     * NOTE: We have to use the low level class loader here because the pluginlib::ClassLoader API doesn't allow
     * customizing the internal node/library allocation map.
     */

    // Instantiate loaders for all libraries in library_paths (We don't use class_loader::MultiLibraryClassLoader
    // because we want to keep track of the libraries that the nodes come from for debugging purposes)
    std::vector<std::unique_ptr<class_loader::ClassLoader>> class_loaders;
    for (const auto & path : library_paths) class_loaders.push_back(std::make_unique<class_loader::ClassLoader>(path));

    // Walk manifest and register all plugins with BT::BehaviorTreeFactory
    for (const auto & [node_name, params] : manifest.map()) {
      const std::string required_class_name =
        "auto_apms_behavior_tree::core::NodeRegistrationTemplate<" + params.class_name + ">";

      class_loader::ClassLoader * loader = nullptr;
      for (const auto & l : class_loaders) {
        if (l->isClassAvailable<core::NodeRegistrationInterface>(required_class_name)) loader = l.get();
      }

      if (!loader) {
        throw std::runtime_error(
          "Node '" + node_name + " (Class: " + params.class_name +
          ")' cannot be registered, because the required registration class '" + required_class_name +
          "' couldn't be found. Check that the class name is spelled correctly and "
          "the node is registered by calling auto_apms_behavior_tree_register_nodes() in the CMakeLists.txt of the "
          "corresponding package. Also make sure that you called the "
          "AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE macro in the source file.");
      }

      RCLCPP_DEBUG(
        logger, "Registering behavior tree node '%s' (Class: %s) from library %s.", node_name.c_str(),
        params.class_name.c_str(), loader->getLibraryPath().c_str());

      try {
        const auto plugin_instance = loader->createUniqueInstance<core::NodeRegistrationInterface>(required_class_name);
        rclcpp::Node::SharedPtr node = nullptr;
        rclcpp::CallbackGroup::SharedPtr group = nullptr;
        rclcpp::executors::SingleThreadedExecutor::SharedPtr executor = nullptr;
        core::RosNodeContext ros_node_context(
          node, group, executor, params);  // Values don't matter when not instantiating it
        plugin_instance->registerWithBehaviorTreeFactory(factory, node_name, &ros_node_context);
      } catch (const std::exception & e) {
        throw std::runtime_error(
          "Failed to register node '" + node_name + " (Class: " + params.class_name + ")': " + e.what());
      }
    }

    // Generate and write node model
    std::ofstream out_stream(output_file);
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
