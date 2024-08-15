#include <action_msgs/srv/cancel_goal.hpp>
#include <functional>
#include <px4_behavior/bt_executor_client.hpp>
#include <px4_behavior/definitions.hpp>
#include <px4_behavior/get_resource.hpp>

namespace px4_behavior {

BTExecutorClient::BTExecutorClient(rclcpp::Node& node, const std::string& executor_name)
    : node_base_interface_ptr_{node.get_node_base_interface()},
      logger_{node.get_logger().get_child("bt_executor_client")},
      executor_name_{executor_name},
      upload_service_name_{executor_name + std::string(BT_EXECUTOR_UPLOAD_SERVICE_NAME_SUFFIX)},
      launch_executor_action_name_{executor_name + std::string(BT_EXECUTOR_LAUNCH_ACTION_NAME_SUFFIX)}
{
    upload_client_ptr_ = node.create_client<UploadService>(upload_service_name_);
    launch_client_ptr_ = rclcpp_action::create_client<LaunchExecutorAction>(&node, launch_executor_action_name_);
}

bool BTExecutorClient::UploadBehaviorTree(const std::string& package_name,
                                          const std::string& trees_filename,
                                          const std::string& tree_id)
{
    std::string content;
    try {
        content = px4_behavior::read_behavior_tree_filepath(px4_behavior::get_behavior_tree_filepath(package_name, trees_filename));
    } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(logger_, "UploadBehaviorTree: %s", e.what());
        return false;
    }

    // Remove extra spaces
    auto new_end =
        std::unique(content.begin(), content.end(), [](char a, char b) { return std::isspace(a) && std::isspace(b); });
    content.erase(new_end, content.end());

    return UploadBehaviorTreeFromText(content, tree_id);
}

bool BTExecutorClient::UploadBehaviorTreeFromText(const std::string& xml_data, const std::string& tree_id)
{
    if (!upload_client_ptr_->wait_for_service(WAIT_FOR_SERVER_TIMEOUT)) {
        RCLCPP_ERROR(logger_,
                     "UploadBehaviorTree: Registration service '%s' is not available",
                     upload_service_name_.c_str());
        return false;
    }

    // Send request
    auto request = std::make_shared<UploadService::Request>();
    request->xml_data = xml_data;
    request->tree_id = tree_id;
    auto future = upload_client_ptr_->async_send_request(request);

    // Wait for the response
    switch (rclcpp::spin_until_future_complete(node_base_interface_ptr_, future, UPLOAD_RESPONSE_TIMEOUT)) {
        case rclcpp::FutureReturnCode::SUCCESS: {
            // Extract the response as highlighted here:
            // https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html#client-asnyc-send-request-request-returns-a-std-future-instead-of-a-std-shared-future
            auto response = future.get();

            if (response->success) {
                RCLCPP_INFO(logger_, "UploadBehaviorTree: Registration successful");
                return true;
            }
            RCLCPP_ERROR(logger_, "UploadBehaviorTree: Registration failed: %s", response->error_message.c_str());
            break;
        }
        case rclcpp::FutureReturnCode::TIMEOUT:
            upload_client_ptr_->remove_pending_request(future);
            RCLCPP_ERROR(logger_, "UploadBehaviorTree: Registration timeout");
            break;
        default:
            RCLCPP_ERROR(logger_, "UploadBehaviorTree: Registration failed");
            break;
    }
    return false;
}

std::shared_future<ExecutionResultSharedPtr> BTExecutorClient::RequestLaunch()
{
    auto promise_ptr = std::make_shared<std::promise<ExecutionResultSharedPtr>>();
    if (!launch_client_ptr_->wait_for_action_server(WAIT_FOR_SERVER_TIMEOUT)) {
        RCLCPP_ERROR(logger_,
                     "RequestLaunch: Execution action '%s' is not available",
                     launch_executor_action_name_.c_str());
        promise_ptr->set_value(nullptr);
        return promise_ptr->get_future();
    }

    using namespace std::placeholders;
    auto send_goal_options = rclcpp_action::Client<LaunchExecutorAction>::SendGoalOptions{};

    // Feedback callback
    send_goal_options.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<LaunchExecutorAction>::SharedPtr goal_handle_ptr,
               const std::shared_ptr<const LaunchExecutorAction::Feedback> feedback) {
            (void)goal_handle_ptr;
            root_tree_id_ = feedback->root_tree_id;
            running_tree_action_name_ = feedback->running_action_name;
            running_tree_action_timestamp_ = feedback->running_action_timestamp;
        };

    // Result callback
    send_goal_options.result_callback =
        [this, promise_ptr](const rclcpp_action::ClientGoalHandle<LaunchExecutorAction>::WrappedResult& wr) {
            switch (wr.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(logger_, "Behavior tree '%s' terminated successfully", root_tree_id_.c_str());
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_WARN(logger_,
                                 "Behavior tree '%s' was aborted: %s",
                                 root_tree_id_.c_str(),
                                 wr.result->termination_message.c_str());
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(logger_, "Behavior tree '%s' was canceled", root_tree_id_.c_str());
                    break;
                default:
                    RCLCPP_ERROR(logger_, "Unknown result code");
                    break;
            }
            promise_ptr->set_value(
                std::make_shared<rclcpp_action::ClientGoalHandle<LaunchExecutorAction>::WrappedResult>(wr));
            launch_executor_goal_handle_ = nullptr;
        };

    LaunchExecutorAction::Goal goal;
    auto goal_response_future = launch_client_ptr_->async_send_goal(goal, send_goal_options);

    switch (rclcpp::spin_until_future_complete(node_base_interface_ptr_,
                                               goal_response_future,
                                               START_EXECUTION_GOAL_RESPONSE_TIMEOUT)) {
        case rclcpp::FutureReturnCode::SUCCESS:
            break;
        case rclcpp::FutureReturnCode::TIMEOUT:
            RCLCPP_ERROR(logger_, "RequestLaunch: Goal response timeout");
            promise_ptr->set_value(nullptr);
            return promise_ptr->get_future();
        default:
            RCLCPP_ERROR(logger_, "RequestLaunch: No goal response");
            promise_ptr->set_value(nullptr);
            return promise_ptr->get_future();
    }

    auto client_goal_handle_ptr = goal_response_future.get();
    if (!client_goal_handle_ptr) {
        RCLCPP_ERROR(logger_, "RequestLaunch: Goal was rejected");
        promise_ptr->set_value(nullptr);
        return promise_ptr->get_future();
    }

    launch_executor_goal_handle_ = client_goal_handle_ptr;
    RCLCPP_INFO(logger_, "RequestLaunch: BTExecutor '%s' was started successfully", executor_name_.c_str());
    return promise_ptr->get_future();
}

bool BTExecutorClient::RequestCancelation()
{
    auto cancel_response_future = launch_client_ptr_->async_cancel_goal(launch_executor_goal_handle_);
    switch (rclcpp::spin_until_future_complete(node_base_interface_ptr_,
                                               cancel_response_future,
                                               CANCEL_EXECUTION_RESPONSE_TIMEOUT)) {
        case rclcpp::FutureReturnCode::SUCCESS:
            break;
        case rclcpp::FutureReturnCode::TIMEOUT:
            RCLCPP_ERROR(logger_, "RequestCancelation: Cancel response timeout");
            return false;
        default:
            RCLCPP_ERROR(logger_, "RequestCancelation: No cancel response");
            return false;
    }

    // Evaluate the response message
    switch (cancel_response_future.get()->return_code) {
        case action_msgs::srv::CancelGoal::Response::ERROR_NONE:
            RCLCPP_INFO(logger_, "RequestCancelation: Server ACCEPTED");
            return true;
        case action_msgs::srv::CancelGoal::Response::ERROR_REJECTED:
            RCLCPP_ERROR(logger_, "RequestCancelation: Server REJECTED");
            break;
        case action_msgs::srv::CancelGoal::Response::ERROR_UNKNOWN_GOAL_ID:
            RCLCPP_ERROR(logger_, "RequestCancelation: Goal is unkown");
            break;
        case action_msgs::srv::CancelGoal::Response::ERROR_GOAL_TERMINATED:
            RCLCPP_WARN(logger_, "RequestCancelation: Goal has already terminated");
            return true;
    }
    return false;
}

bool BTExecutorClient::has_launched() { return !!launch_executor_goal_handle_; }

const std::string& BTExecutorClient::root_tree_id() { return root_tree_id_; }

const std::string& BTExecutorClient::running_tree_action_name() { return running_tree_action_name_; }

double BTExecutorClient::running_tree_action_timestamp() { return running_tree_action_timestamp_; }

}  // namespace px4_behavior
