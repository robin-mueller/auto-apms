#include <signal.h>

#include <chrono>
#include <filesystem>

#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "px4_behavior/bt_factory.hpp"
#include "rclcpp/rclcpp.hpp"

sig_atomic_t volatile shutdown_requested = 0;

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    // Ensure that rclcpp is not shut down before the tree has been halted (on destruction) and all pending actions have
    // been successfully canceled
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
    signal(SIGINT, [](int sig) {
        (void)sig;
        shutdown_requested = 1;
    });
    auto node = std::make_shared<rclcpp::Node>(std::string(EXAMPLE_NAME) + "_node");

    {
        BT::Tree tree =
            px4_behavior::CreateBehaviorTreeFromResource(node, "relative_goto", std::nullopt, "px4_behavior_examples");
        if (!tree.rootNode()) {
            std::cerr << "Tree couldn't be created\n";
            return EXIT_FAILURE;
        }

        BT::Groot2Publisher publisher(tree);
        BT::StdCoutLogger logger(tree);
        logger.enableTransitionToIdle(false);

        auto status = BT::NodeStatus::IDLE;
        while (!shutdown_requested && status <= BT::NodeStatus::RUNNING) {
            status = tree.tickOnce();
            tree.sleep(1ms);
        }

        std::cout << "Finished tree with status: " << BT::toStr(status) << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}