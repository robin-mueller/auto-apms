#include "px4_behavior/bt_executor_client.hpp"

enum class TextColor { GREEN, RED, YELLOW, BLUE, MAGENTA, CYAN };

std::string colored(const std::string& text, TextColor color)
{
    switch (color) {
        case TextColor::RED:
            return "\x1b[31m" + text + "\x1b[0m";
        case TextColor::GREEN:
            return "\x1b[32m" + text + "\x1b[0m";
        case TextColor::YELLOW:
            return "\x1b[33m" + text + "\x1b[0m";
        case TextColor::BLUE:
            return "\x1b[34m" + text + "\x1b[0m";
        case TextColor::MAGENTA:
            return "\x1b[35m" + text + "\x1b[0m";
        case TextColor::CYAN:
            return "\x1b[36m" + text + "\x1b[0m";
    }
    return "undefined";
}

using namespace px4_behavior;

int main(int argc, char* argv[])
{
    if (argc < 5) {
        std::cerr << "upload_tree: Missing inputs! The program requires: \n\t1.) the namespace of the executor node\n\t"
                     "2.) the name of the executor to register behavior trees with\n\t3.) the name of the package that "
                     "provides the behavior trees file\n\t4.) the name of the file containing the xml data\n\t5.) "
                     "Optional: the ID of the tree to execute\n";
        std::cerr
            << "Usage: register_tree <namespace> <executor_name> <package_name> <tree_file_name> [<main_tree_id>]\n";
        return EXIT_FAILURE;
    }
    const std::string namespace_{argv[1]};
    const std::string executor_name{argv[2]};
    const std::string package_name{argv[3]};
    const std::string tree_file_name{argv[4]};
    const std::string main_tree_id{argc > 5 ? argv[5] : ""};

    std::cout << "Uploading behavior tree to executor '" << colored(executor_name, TextColor::CYAN)
              << "' in namespace '" << colored(namespace_, TextColor::CYAN) << "'"
              << "\n\tpackage_name  \t'" << colored(package_name, TextColor::CYAN) << "'"
              << "\n\ttrees_filename\t'" << colored(tree_file_name, TextColor::CYAN) << "'"
              << "\n\ttree_id       \t'"
              << colored(main_tree_id.empty() ? "[main_tree_to_execute]" : main_tree_id, TextColor::CYAN) << "'"
              << std::endl;

    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<rclcpp::Node>(executor_name + "_upload_node", namespace_);

    // Restrict ROS2 logging output
    auto ret = rcutils_logging_set_logger_level(node_ptr->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);
    if (ret != RCUTILS_RET_OK) {
        std::cerr << "Error setting ROS2 logging severity: " << rcutils_get_error_string().str << '\n';
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    // Create behavior tree executor client
    auto bt_executor_client = BTExecutorClient(*node_ptr, executor_name);

    // Register all behavior trees that are defined in a file with the executor
    auto resource = FetchBehaviorTreeResource(tree_file_name, std::nullopt, package_name);
    if (resource.has_value()) {
        if (bt_executor_client.UploadBehaviorTreeFromResource(resource.value(), main_tree_id)) {
            std::cout << " --> " << colored("Registration successful", TextColor::GREEN) << std::endl;
        }
        else {
            std::cout << " --> " << colored("Registration failed", TextColor::RED) << std::endl;
        }
    }
    else {
        std::cerr << "No matching behvior tree resource could be found\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
