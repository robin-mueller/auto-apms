#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_behavior/ros2_bt_observer.hpp>
#include <px4_behavior_interfaces/action/bt_executor_command.hpp>
#include <px4_behavior_interfaces/action/launch_bt_executor.hpp>
#include <px4_behavior_interfaces/srv/upload_behavior_tree.hpp>

namespace px4_behavior {

using LaunchExecutorAction = px4_behavior_interfaces::action::LaunchBTExecutor;
using UploadService = px4_behavior_interfaces::srv::UploadBehaviorTree;
using CommandAction = px4_behavior_interfaces::action::BTExecutorCommand;

enum class BTExecutorState : uint8_t { IDLE, RUNNING, PAUSED, HALTED, TERMINATED };
enum class BTExecutorCommand : uint8_t { RUN, PAUSE, HALT, TERMINATE };

std::string to_string(BTExecutorState state);
std::string to_string(BTExecutorCommand cmd);

class BTExecutor
{
   protected:
    using State = BTExecutorState;
    using Command = BTExecutorCommand;

    enum class ClosureConduct : uint8_t { SUCCEED, ABORT, RESTART };

    BTExecutor(const std::string& name,
               const rclcpp::NodeOptions& options,
               const int groot2_server_port = 5555,
               const std::chrono::milliseconds& bt_tick_interval = std::chrono::milliseconds{100});

   public:
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

    virtual void SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory) = 0;
    virtual void OnTreeCreated(BT::Blackboard& global_blackboard);
    virtual Command ReviewControlCommand(Command current_command, State current_state);
    virtual void BeforeFirstTick(BT::Blackboard& global_blackboard);
    virtual ClosureConduct OnResult(bool success);

   private:
    /**
     *  Services
     */
    void UploadBehaviorTree(const std::shared_ptr<UploadService::Request> request,
                            std::shared_ptr<UploadService::Response> response);

    /**
     *  LaunchBTExecutor action
     */
    rclcpp_action::GoalResponse LaunchHandleGoal(const rclcpp_action::GoalUUID& uuid,
                                                 std::shared_ptr<const LaunchExecutorAction::Goal> goal_ptr);
    rclcpp_action::CancelResponse LaunchHandleCancel(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<LaunchExecutorAction>> goal_handle_ptr);
    void LaunchHandleAccepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<LaunchExecutorAction>> goal_handle_ptr);

    /**
     *  BTExecutorCommand action
     */
    rclcpp_action::GoalResponse CommandHandleGoal(const rclcpp_action::GoalUUID& uuid,
                                                  std::shared_ptr<const CommandAction::Goal> goal_ptr);
    rclcpp_action::CancelResponse CommandHandleCancel(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<CommandAction>> goal_handle_ptr);
    void CommandHandleAccepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<CommandAction>> goal_handle_ptr);

   protected:
    // Empty tree_id means to use the main_tree_to_execute flag in the XML data
    void CreateTree(const std::string& tree_id = "");
    State GetExecutionState();
    std::string GetCreatedTreeID();
    bool HasLaunched();
    void SetUpExecutionRoutine();

    rclcpp::Node::SharedPtr node();
    BT::Blackboard::Ptr global_blackboard();

   private:
    rclcpp::Node::SharedPtr node_ptr_;
    const int groot2_server_port_;
    const std::chrono::milliseconds bt_tick_interval_;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_ptr_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> state_change_logging_param_handle_ptr_;
    rclcpp_action::Server<LaunchExecutorAction>::SharedPtr launch_action_ptr_;
    rclcpp::Service<UploadService>::SharedPtr upload_service_ptr_;
    rclcpp_action::Server<CommandAction>::SharedPtr command_action_ptr_;
    rclcpp::TimerBase::SharedPtr execution_timer_ptr_{nullptr};
    rclcpp::TimerBase::SharedPtr command_request_timer_ptr_{nullptr};
    std::weak_ptr<rclcpp_action::ServerGoalHandle<LaunchExecutorAction>> launch_goal_handle_ptr_;

    BT::Blackboard::Ptr global_blackboard_ptr_;
    std::unique_ptr<BT::BehaviorTreeFactory> bt_factory_ptr_{nullptr};
    std::unique_ptr<BT::Tree> behavior_tree_ptr_{nullptr};
    std::unique_ptr<BT::Groot2Publisher> bt_groot2_publisher_ptr_{nullptr};
    std::unique_ptr<BTStateObserver> bt_state_observer_ptr_{nullptr};
    Command control_command_;
    bool is_paused_{true};  // Needs to be true for the state to evaluate to IDLE in the first iteration
    LaunchExecutorAction::Feedback last_feedback_;
};

}  // namespace px4_behavior
