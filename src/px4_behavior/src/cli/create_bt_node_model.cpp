/**
 * \file
 * \brief Command line tool to create a model of the used node plugins
 */

#include <behaviortree_cpp/xml_parsing.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <px4_behavior/factory.hpp>

int main(int argc, char** argv)
{
    using namespace px4_behavior;

    if (argc < 3) {
        std::cerr
            << "create_bt_node_model: Missing inputs! The program requires: \n\t1.) the yaml configuration file to "
               "pass to px4_behavior::RegisterNodePlugins\n\t2.) the xml file to store the model\n\t3.) Optional: the "
               "directory that contains additional plugins that cannot be found under the package install "
               "directories\n";
        std::cerr << "Usage: create_bt_node_model <input_file> <output_file> [<extra_plugin_dir>]\n";
        return EXIT_FAILURE;
    }
    std::filesystem::path config_file{argv[1]};
    std::filesystem::path output_file{argv[2]};
    std::optional<std::filesystem::path> extra_plugin_dir = std::nullopt;
    if (argc > 3) {
        extra_plugin_dir = std::filesystem::path{argv[3]};
        if ((*extra_plugin_dir).empty()) extra_plugin_dir = std::nullopt;
    }

    // Check if config exists
    if (!std::filesystem::exists(config_file)) {
        throw std::runtime_error("Config file '" + config_file.string() + "' doesn't exist");
    }

    // Check if plugin dir exists
    if (extra_plugin_dir.has_value() && !std::filesystem::is_directory(*extra_plugin_dir)) {
        throw std::runtime_error("Plugin directory '" + (*extra_plugin_dir).string() + "' doesn't exist");
    }

    // Ensure correct extensions
    if (config_file.extension().compare(".yaml") != 0) {
        throw std::runtime_error("Config file '" + config_file.string() + "' has wrong extension. Must be '.yaml'");
    }
    if (output_file.extension().compare(".xml") != 0) {
        throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.xml'");
    }

    std::cout << "create_bt_node_model: \n\tInput is " + config_file.string() + "\n\tOutput will be " +
                     output_file.string()
              << std::endl;
    if (extra_plugin_dir.has_value()) {
        std::cout << "\n\tPlugin directory is " + (*extra_plugin_dir).string() << std::endl;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("_create_bt_node_model_temp_node");
    BT::BehaviorTreeFactory factory;

    if (RegisterNodePlugins(factory, node, config_file, extra_plugin_dir) != RegistrationStatus::SUCCESS) {
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
