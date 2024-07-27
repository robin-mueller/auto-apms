#include <signal.h>

#include <px4_behavior/bt_executor_client.hpp>

sig_atomic_t volatile shutdown_requested = 0;

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
    if (argc < 3) {
        std::cerr << "run_executor: Missing inputs! The program requires: \n\t1.) the namespace ot the executor "
                     "node\n\t2.) the name of the executor\n";
        std::cerr << "Usage: run_executor <namespace> <executor_name>\n";
        return EXIT_FAILURE;
    }
    const std::string namespace_{argv[1]};
    const std::string executor_name{argv[2]};

    std::cout << "Starting Behavior Tree Executor '" << colored(executor_name, TextColor::CYAN) << "' in namespace '"
              << colored(namespace_, TextColor::CYAN) << "'" << std::endl;

    // Ensure that rclcpp is not shut down before the tree has terminated
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
    signal(SIGINT, [](int sig) {
        (void)sig;
        shutdown_requested = 1;
    });
    auto node_ptr = std::make_shared<rclcpp::Node>(executor_name + "_run_executor_node", namespace_);

    // Restrict ROS2 logging output
    auto ret = rcutils_logging_set_logger_level(node_ptr->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);
    if (ret != RCUTILS_RET_OK) {
        std::cerr << "Error setting ROS2 logging severity: " << rcutils_get_error_string().str << '\n';
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    // Create behavior tree executor client
    auto bt_executor_client = BTExecutorClient(*node_ptr, executor_name);

    // Request tree execution
    auto execution_result_future = bt_executor_client.RequestLaunch();

    // Check if the request failed before proceeding
    auto status = execution_result_future.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
        if (!execution_result_future.get()) {
            std::cerr << "Failed to request execution\n";
            rclcpp::shutdown();
            return EXIT_FAILURE;
        }
    }

    std::cout << " --> " << colored("Behavior tree is executing", TextColor::GREEN) << std::endl;
    std::cout << "\nBehavior tree execution log:" << std::endl;

    // Spin node as long as tree has not terminated while checking for ctrl+c signal
    bool cancelation_requested = false;
    double last_running_node_timestamp = 0;
    while (rclcpp::spin_until_future_complete(node_ptr, execution_result_future, std::chrono::milliseconds(1)) !=
           rclcpp::FutureReturnCode::SUCCESS) {
        if (shutdown_requested && !cancelation_requested) {
            if (!bt_executor_client.RequestCancelation()) {
                std::cerr << "Failed to cancel behavior tree\n";
                rclcpp::shutdown();
                return EXIT_FAILURE;
            }
            cancelation_requested = true;
        }

        // Print feedback
        if (bt_executor_client.running_tree_action_timestamp() > last_running_node_timestamp) {
            last_running_node_timestamp = bt_executor_client.running_tree_action_timestamp();
            std::cout << "\t[" << bt_executor_client.root_tree_id() << "] "
                      << bt_executor_client.running_tree_action_name() << std::endl;
        }
    }
    auto execution_result_ptr = execution_result_future.get();
    auto terminated_tree_id = execution_result_ptr->result->terminated_tree_id;

    switch (execution_result_ptr->code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            switch (execution_result_ptr->result->tree_result) {
                case LaunchExecutorAction::Result::TREE_RESULT_SUCCESS:
                    std::cout << "\n --> "
                              << colored("Behavior tree with ID '", TextColor::GREEN) +
                                     colored(terminated_tree_id, TextColor::CYAN) +
                                     colored("' succeeded", TextColor::GREEN)
                              << std::endl;
                    break;
                case LaunchExecutorAction::Result::TREE_RESULT_FAILURE:
                    std::cout << "\n --> "
                              << colored("Behavior tree with ID '", TextColor::RED) +
                                     colored(terminated_tree_id, TextColor::CYAN) + colored("' failed", TextColor::RED)
                              << std::endl;
                    break;
                case LaunchExecutorAction::Result::TREE_RESULT_NOT_SET:
                    std::cout << "\n --> "
                              << colored("Execution of behavior tree with ID '", TextColor::YELLOW) +
                                     colored(terminated_tree_id, TextColor::CYAN) +
                                     colored("' succeeded, but no result was specified", TextColor::YELLOW)
                              << std::endl;
                    break;
            }
            break;
        case rclcpp_action::ResultCode::CANCELED:
            std::cout << "\n --> "
                      << colored("Behavior tree with ID '", TextColor::YELLOW) +
                             colored(terminated_tree_id, TextColor::CYAN) + colored("' was halted", TextColor::YELLOW)
                      << std::endl;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            std::cout << "\n --> "
                      << colored("Execution of behavior tree with ID '", TextColor::RED) +
                             colored(terminated_tree_id, TextColor::CYAN) +
                             colored(
                                 "' terminated unexpectedly!\n\n" + execution_result_ptr->result->termination_message,
                                 TextColor::RED)
                      << std::endl;
            break;
        default:
            std::cout << "\n --> Behavior tree with ID '" << terminated_tree_id << "' terminated with code "
                      << static_cast<uint8_t>(execution_result_ptr->code) << std::endl;
            break;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
