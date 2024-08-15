#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <signal.h>

#include <chrono>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <px4_behavior/factory.hpp>
#include <px4_behavior/get_resource.hpp>

sig_atomic_t volatile shutdown_requested = 0;

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    const auto config_filepath =
        px4_behavior::get_plugin_config_filepath("px4_behavior", std::string(EXAMPLE_NAME) + "_bt_node_config");
    const auto tree_filepath = px4_behavior::get_behavior_tree_filepath("px4_behavior", std::string(EXAMPLE_NAME) + "_tree");

    // Ensure that rclcpp is not shut down before the tree has been halted (on destruction) and all pending actions have
    // been successfully canceled
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::SigTerm);
    signal(SIGINT, [](int sig) {
        (void)sig;
        shutdown_requested = 1;
    });
    auto node = std::make_shared<rclcpp::Node>(std::string(EXAMPLE_NAME) + "_node");

    {
        BT::BehaviorTreeFactory factory;
        px4_behavior::RegisterNodePlugins(factory, node, config_filepath);
        auto tree = factory.createTreeFromFile(tree_filepath);

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