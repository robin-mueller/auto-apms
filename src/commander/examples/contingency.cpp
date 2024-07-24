#include <chrono>
#include <commander/vehicle_command_client.hpp>
#include <commander_interfaces/action/rtl.hpp>
#include <future>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    // Create vehicle_command_client
    auto node_ptr = std::make_shared<rclcpp::Node>(std::string(EXAMPLE_NAME) + "_node");
    auto vehicle_command_client = commander::VehicleCommandClient(*node_ptr);

    // Enable debug output
    auto ret = rcutils_logging_set_logger_level(node_ptr->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_ERROR(node_ptr->get_logger(), "Error setting severity: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }

    RCLCPP_INFO(node_ptr->get_logger(), "Running example '%s' ...", EXAMPLE_NAME);

    // Create RTL action
    // IMPORTANT: Make sure to launch the required maneuver first!
    using RTLActionType = commander_interfaces::action::RTL;
    RTLActionType::Result::SharedPtr result = nullptr;
    const std::string action_name = "rtl";
    auto action_client = rclcpp_action::create_client<RTLActionType>(node_ptr, action_name);
    if (!action_client->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_ptr->get_logger(), "There is no action with name '%s'", action_name.c_str());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_ptr);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    RCLCPP_INFO(node_ptr->get_logger(), "Starting mission ...");
    vehicle_command_client.StartMission();

    std::this_thread::sleep_for(5s);  // Wait shortly before sending RTL request

    auto send_goal_options = rclcpp_action::Client<RTLActionType>::SendGoalOptions{};
    send_goal_options.result_callback =
        [&result, &node_ptr](const rclcpp_action::ClientGoalHandle<RTLActionType>::WrappedResult& r) {
            switch (r.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    result = r.result;
                    RCLCPP_INFO(node_ptr->get_logger(), "Goal succeeded.");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(node_ptr->get_logger(), "Goal was aborted.");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(node_ptr->get_logger(), "Goal was canceled.");
                    break;
                default:
                    RCLCPP_ERROR(node_ptr->get_logger(), "Unknown result code.");
                    break;
            }
        };

    RCLCPP_INFO(node_ptr->get_logger(), "Sending RTL action goal ...");
    auto goal = RTLActionType::Goal{};
    auto client_goal_handle_future = action_client->async_send_goal(goal, send_goal_options);

    while (client_goal_handle_future.wait_for(1s) != std::future_status::ready) {
        RCLCPP_INFO(node_ptr->get_logger(), "Waiting for result");
        if (!rclcpp::ok()) {
            rclcpp::shutdown();
            spinner.join();
            return 2;
        };
    }

    auto client_goal_handle = client_goal_handle_future.get();

    if (!client_goal_handle) { throw std::runtime_error("The goal was rejected."); }

    // Wait while the goal is executing
    int8_t status;
    do {
        std::this_thread::sleep_for(1s);
        status = client_goal_handle->get_status();
        RCLCPP_INFO(node_ptr->get_logger(), "Goal status: %i", status);
        if (!rclcpp::ok()) {
            rclcpp::shutdown();
            spinner.join();
            return 3;
        }
    } while (status == rclcpp_action::GoalStatus::STATUS_EXECUTING);

    // Make sure to cleanly deactivate the mode executor on completion
    vehicle_command_client.SyncActivateFlightMode(commander::VehicleCommandClient::FlightMode::Hold);

    RCLCPP_INFO(node_ptr->get_logger(), "Shutting down.");

    rclcpp::shutdown();
    spinner.join();

    return 0;
}
