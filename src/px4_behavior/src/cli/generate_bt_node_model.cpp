/**
 * \file
 * \brief Command line tool to generate a model of specific behavior tree node plugins
 */

#include <behaviortree_cpp/xml_parsing.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <px4_behavior/bt_factory.hpp>

int main(int argc, char** argv)
{
    using namespace px4_behavior;

    if (argc < 3) {
        std::cerr
            << "generate_bt_node_model: Missing inputs! The program requires: \n\t1.) the yaml configuration file to "
               "pass to px4_behavior::RegisterBTNodePlugins\n\t2.) the xml file to store the model\n\t3.) Optional: "
               "additional build information for plugins of the same package that issues the model generation\n";
        std::cerr << "Usage: generate_bt_node_model <input_file> <output_file> [<build_info>...]\n";
        return EXIT_FAILURE;
    }
    std::filesystem::path config_file{argv[1]};
    std::filesystem::path output_file{argv[2]};
    std::vector<std::string> build_infos = {};
    for (int i = 3; i < argc; ++i) { build_infos.push_back(argv[i]); }

    // Check if config exists
    if (!std::filesystem::exists(config_file)) {
        throw std::runtime_error("Config file '" + config_file.string() + "' doesn't exist");
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

#ifdef DEBUG_LOGGING
    // Set logging severity
    auto ret = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(node->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }
#endif

    BT::BehaviorTreeFactory factory;

    if (RegisterBTNodePlugins(factory, node, config_file, build_infos) != RegistrationStatus::SUCCESS) {
        std::cerr << "generate_bt_node_model: Error registering node plugins with config "
                  << std::filesystem::absolute(config_file) << "\n";
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
