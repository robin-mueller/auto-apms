/**
 * \file
 * \brief Command line tool to generate a model of specific behavior tree node plugins
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
            << "generate_bt_node_model: Missing inputs! The program requires: \n\t1.) the yaml configuration file to "
               "pass to px4_behavior::RegisterNodePlugins\n\t2.) the xml file to store the model\n\t3.) Optional: "
               "additional plugins that cannot be found under the package share directories\n";
        std::cerr << "Usage: generate_bt_node_model <input_file> <output_file> [<extra_plugins>...]\n";
        return EXIT_FAILURE;
    }
    std::filesystem::path config_file{argv[1]};
    std::filesystem::path output_file{argv[2]};
    std::vector<std::filesystem::path> extra_plugins = {};
    for (int i = 3; i < argc; ++i) { extra_plugins.push_back(std::filesystem::path{argv[i]}); }

    // Check if config exists
    if (!std::filesystem::exists(config_file)) {
        throw std::runtime_error("Config file '" + config_file.string() + "' doesn't exist");
    }

    // Check if plugin dir exists
    for (const auto& plugin_path : extra_plugins) {
        if (!std::filesystem::exists(plugin_path)) {
            throw std::runtime_error("Extra plugin '" + plugin_path.string() + "' doesn't exist");
        }
    }

    // Ensure correct extensions
    if (config_file.extension().compare(".yaml") != 0) {
        throw std::runtime_error("Config file '" + config_file.string() + "' has wrong extension. Must be '.yaml'");
    }
    if (output_file.extension().compare(".xml") != 0) {
        throw std::runtime_error("Output file '" + output_file.string() + "' has wrong extension. Must be '.xml'");
    }

    std::cout << "generate_bt_node_model: \n\tInput is " + std::filesystem::absolute(config_file).string() +
                     "\n\tOutput will be " + std::filesystem::absolute(output_file).string()
              << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("_generate_bt_node_model_temp_node");
    BT::BehaviorTreeFactory factory;

    if (RegisterNodePlugins(factory, node, config_file, extra_plugins) != RegistrationStatus::SUCCESS) {
        std::cerr << "generate_bt_node_model: Error registering node plugins\n";
        return EXIT_FAILURE;
    }

    std::ofstream out_stream{output_file};
    if (out_stream.is_open()) {
        out_stream << BT::writeTreeNodesModelXML(factory);
        out_stream.close();
    }
    else {
        std::cerr << "generate_bt_node_model: Error opening tree nodes model output file '" << output_file << "'\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
