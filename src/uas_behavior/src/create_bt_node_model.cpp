/**
 * \file
 * \brief Command line tool to create a model of the used node plugins
 */

#include <behaviortree_cpp/xml_parsing.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <uas_behavior/factory.hpp>

int main(int argc, char** argv)
{
    using namespace uas_behavior;

    if (argc < 4) {
        std::cerr
            << "create_bt_node_model: Missing inputs! The program requires: \n\t1.) the yaml configuration file to "
               "pass to uas_behavior::RegisterNodePlugins\n\t2.) the xml file to store the model\n\t3.) the "
               "directory that contains the plugins specified in the configuration file\n";
        std::cerr << "Usage: create_bt_node_model <input_file> <output_file> <plugin_dir>\n";
        return EXIT_FAILURE;
    }
    std::filesystem::path config_file{argv[1]};
    std::filesystem::path output_file{argv[2]};
    std::filesystem::path plugin_directory{argv[3]};

    // Check if config exists
    if (!std::filesystem::exists(config_file)) {
        throw std::runtime_error("Config file '" + config_file.string() + "' doesn't exist");
    }

    // Check if plugin dir exists
    if (!std::filesystem::is_directory(plugin_directory)) {
        throw std::runtime_error("Plugin directory '" + plugin_directory.string() + "' doesn't exist");
    }

    // Ensure correct extensions
    if (config_file.extension().compare(".yaml") != 0) {
        throw std::runtime_error("Config file '" + config_file.string() + "' has wrong extension. Must be '.yaml'");
    }
    if (output_file.extension().compare(".xml") != 0) {
        throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.xml'");
    }

    std::cout << "create_bt_node_model: \n\tInput is " + config_file.string() + "\n\tOutput will be " +
                     output_file.string() + "\n\tPlugin directory is " + plugin_directory.string()
              << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("_create_bt_node_model_temp_node");
    BT::BehaviorTreeFactory factory;

    if (RegisterNodePlugins(factory, node, config_file, plugin_directory) != RegistrationStatus::SUCCESS) {
        std::cerr << "create_bt_node_model: Error registering node plugins\n";
        return EXIT_FAILURE;
    }

    std::ofstream out_stream{output_file};
    if (out_stream.is_open()) {
        out_stream << BT::writeTreeNodesModelXML(factory);
        out_stream.close();
    }
    else {
        std::cerr << "create_bt_node_model: Error opening tree nodes model output file '" << output_file << "'\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
