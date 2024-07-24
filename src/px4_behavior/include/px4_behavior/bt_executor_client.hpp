#pragma once

#include <chrono>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_behavior_interfaces/action/launch_bt_executor.hpp>
#include <px4_behavior_interfaces/srv/upload_behavior_tree.hpp>

namespace px4_behavior {

using LaunchExecutorAction = px4_behavior_interfaces::action::LaunchBTExecutor;
using UploadService = px4_behavior_interfaces::srv::UploadBehaviorTree;
using ExecutionResultSharedPtr = std::shared_ptr<rclcpp_action::ClientGoalHandle<LaunchExecutorAction>::WrappedResult>;

class BTExecutorClient
{
    const std::chrono::seconds WAIT_FOR_SERVER_TIMEOUT{3};
    const std::chrono::seconds UPLOAD_RESPONSE_TIMEOUT{5};
    const std::chrono::seconds START_EXECUTION_GOAL_RESPONSE_TIMEOUT{3};
    const std::chrono::seconds CANCEL_EXECUTION_RESPONSE_TIMEOUT{1};

   public:
    BTExecutorClient(rclcpp::Node& node, const std::string& executor_name);

    /**
     * \brief Upload a behavior tree to an executor.
     *
     * This method is synchronous, meaning that it blocks until upload result is received.
     *
     * \param package_name Name of the package where the trees file is to be found
     * \param trees_filename Name of the trees file (Extension may be omitted)
     * \return True on successful upload, false otherwise.
     */
    bool UploadBehaviorTree(const std::string& package_name,
                            const std::string& trees_filename,
                            const std::string& tree_id = "");

    /**
     * \brief Upload behavior tree to an executor.
     *
     * This method is synchronous, meaning that it blocks until upload result is received.
     *
     * \param xml_data The XML string containing the trees to register
     * \param tree_id ID of the tree to be created on upload. Empty if main_tree_to_execute XML attribute should be used
     * to determine which tree is to be created
     *  \return True on successful upload, false otherwise.
     */
    bool UploadBehaviorTreeFromText(const std::string& xml_data, const std::string& tree_id = "");

    /**
     * \brief Request to launch a behavoir tree executor.
     *
     * \return Shared future that completes when the process terminates, holding the result. Is nullptr if request
     * failed.
     */
    std::shared_future<ExecutionResultSharedPtr> RequestLaunch();

    /**
     * \brief Request to cancel the current behavior execution process.
     *
     * This method is synchronous, meaning that it blocks until the cancelation result is received.
     *
     * \return True if execution was cancelled successfully, false otherwise.
     */
    bool RequestCancelation();

    bool has_launched();
    const std::string& root_tree_id();
    const std::string& running_tree_action_name();
    double running_tree_action_timestamp();

   private:
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_ptr_;
    const rclcpp::Logger logger_;
    const std::string executor_name_;
    const std::string upload_service_name_;
    const std::string launch_executor_action_name_;
    rclcpp::Client<UploadService>::SharedPtr upload_client_ptr_;
    rclcpp_action::Client<LaunchExecutorAction>::SharedPtr launch_client_ptr_;
    rclcpp_action::ClientGoalHandle<LaunchExecutorAction>::SharedPtr launch_executor_goal_handle_{nullptr};

    std::string root_tree_id_;
    std::string running_tree_action_name_;
    double running_tree_action_timestamp_{0};
};

}  // namespace px4_behavior
