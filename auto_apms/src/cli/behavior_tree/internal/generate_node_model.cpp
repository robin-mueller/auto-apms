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

#include "auto_apms/behavior_tree/node_plugin_loader.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "rcpputils/split.hpp"

using namespace auto_apms;

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::cerr
            << "generate_node_model: Missing inputs! The program requires: \n\t1.) The path to the node plugin "
               "manifest.\n\t2.) Paths to shared libraries to be loaded.\n\t3.) The xml file to store the model.\n";
        std::cerr << "Usage: generate_node_model <manifest_file> <library_paths> <output_file>.\n";
        return EXIT_FAILURE;
    }

    try {
        const std::string manifest_file{std::filesystem::absolute(argv[1]).string()};
        const std::vector<std::string> library_paths = rcpputils::split(argv[2], ';');
        const std::filesystem::path output_file{std::filesystem::absolute(argv[3])};

        // Ensure that arguments are not empty
        if (manifest_file.empty()) { throw std::runtime_error("Argument manifest_file must not be empty."); }
        if (library_paths.empty()) { throw std::runtime_error("Argument library_paths must not be empty."); }
        if (output_file.empty()) { throw std::runtime_error("Argument output_file must not be empty."); }

        // Ensure correct extensions
        if (output_file.extension() != ".xml") {
            throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.xml'.");
        }

        // Load libraries
        auto class_loader = std::make_unique<class_loader::MultiLibraryClassLoader>(false);
        for (const auto& path : library_paths) class_loader->loadLibrary(path);

        rclcpp::init(argc, argv);
        auto node = std::make_shared<rclcpp::Node>("_generate_node_model_temp_node");

#ifdef DEBUG_LOGGING
        // Set logging severity
        auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(node->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
            rcutils_reset_error();
        }
#endif

        // Create and write the behavior tree model file
        BT::BehaviorTreeFactory factory;
        const auto manifest = BTNodePluginLoader::Manifest::FromFile(manifest_file);
        BTNodePluginLoader::Load(node, manifest, factory, *class_loader);
        std::ofstream out_stream{output_file};
        if (out_stream.is_open()) {
            out_stream << BT::writeTreeNodesModelXML(factory);
            out_stream.close();
        }
        else {
            throw std::runtime_error("Error opening node model output file '" + output_file.string() + "'.");
        }

    } catch (const std::exception& e) {
        std::cerr << "ERROR (generate_node_model): " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
