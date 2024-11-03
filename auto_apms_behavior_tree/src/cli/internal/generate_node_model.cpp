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

#include "behaviortree_cpp/xml_parsing.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "auto_apms_core/logging.hpp"
#include "auto_apms_behavior_tree/node/node_manifest.hpp"

using namespace auto_apms_behavior_tree;

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "generate_node_model: Missing inputs! The program requires: \n\t1.) The path to the node plugin "
                 "manifest.\n\t2.) The xml file to store the model.\n";
    std::cerr << "Usage: generate_node_model <manifest_file> <output_file>.\n";
    return EXIT_FAILURE;
  }

  try
  {
    const std::string manifest_file{ std::filesystem::absolute(argv[1]).string() };
    const std::filesystem::path output_file{ std::filesystem::absolute(argv[2]) };

    // Ensure that arguments are not empty
    if (manifest_file.empty())
    {
      throw std::runtime_error("Argument manifest_file must not be empty.");
    }
    if (output_file.empty())
    {
      throw std::runtime_error("Argument output_file must not be empty.");
    }

    // Ensure correct extensions
    if (output_file.extension() != ".xml")
    {
      throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.xml'.");
    }

    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<rclcpp::Node>("_generate_node_model_temp_node");
    auto_apms_core::exposeToDebugLogging(node_ptr->get_logger());

    // Create manifest
    BT::BehaviorTreeFactory factory;
    const auto manifest = NodeManifest::fromFile(manifest_file);

    /**
     * NOTE: We have to use the low level class loader here because the pluginlib::ClassLoader API doesn't allow
     * customizing the internal node/library allocation map.
     */

    auto class_loader = class_loader::MultiLibraryClassLoader{ false };
    for (const auto& [node_name, params] : manifest.getInternalMap())
    {
      if (params.library.empty())
      {
        // Library path is a required field now
        throw std::runtime_error("Parameters for node '" + node_name + "' do not specify a library path.");
      }
      const auto& library_path = params.library;

      // Make sure that library is loaded
      if (!class_loader.isLibraryAvailable(library_path))
      {
        try
        {
          class_loader.loadLibrary(library_path);
        }
        catch (const std::exception& e)
        {
          throw std::runtime_error("Failed to load library '" + library_path + "': " + e.what() + ".");
        }
      }

      // Look if the class we search for is actually present in the library.
      const std::string factory_classname =
          "auto_apms_behavior_tree::NodeRegistrationFactory<" + params.class_name + ">";
      const auto classes = class_loader.getAvailableClassesForLibrary<NodeRegistrationInterface>(library_path);
      if (std::find(classes.begin(), classes.end(), factory_classname) == classes.end())
      {
        throw std::runtime_error{ "Node '" + node_name + " (Class: " + params.class_name +
                                  ")' cannot be loaded, because factory class '" + factory_classname +
                                  "' couldn't be found. Check that the class name is spelled correctly and registered "
                                  "by calling auto_apms_behavior_tree_register_nodes() in the CMakeLists.txt of the "
                                  "corresponding package. Also make sure that you called the "
                                  "AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE macro in the source file." };
      }

      RCLCPP_DEBUG(node_ptr->get_logger(), "Loading behavior tree node '%s' (Class: %s) from library %s.",
                   node_name.c_str(), params.class_name.c_str(), library_path.c_str());

      try
      {
        const auto plugin_instance = class_loader.createUniqueInstance<NodeRegistrationInterface>(factory_classname);
        RosNodeContext ros_node_context(node_ptr, params);  // Values don't matter when not instantiating it
        plugin_instance->registerWithBehaviorTreeFactory(factory, node_name, &ros_node_context);
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("Failed to load and register node '" + node_name + " (Class: " + params.class_name +
                                 ")': " + e.what() + ".");
      }
    }

    // Generate and write node model
    std::ofstream out_stream{ output_file };
    if (out_stream.is_open())
    {
      out_stream << BT::writeTreeNodesModelXML(factory);
      out_stream.close();
    }
    else
    {
      throw std::runtime_error("Error opening node model output file '" + output_file.string() + "'.");
    }
  }
  catch (const std::exception& e)
  {
    std::cerr << "ERROR (generate_node_model): " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
