#include <contingency_manager/definitions.hpp>
#include <contingency_manager_interfaces/msg/contingency_event.hpp>
#include <uas_behavior/bt_executor.hpp>
#include <uas_behavior/factory.hpp>

#define KEY_EVENT_ID "event_id"
#define KEY_EVENT_NAME "event_name"

using ContingencyEventMsg = contingency_manager_interfaces::msg::ContingencyEvent;

namespace contingency_manager {

class ContingencyManagerExecutor : public uas_behavior::BTExecutor
{
   public:
    ContingencyManagerExecutor(const rclcpp::NodeOptions& options);

   private:
    void SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory) final;
    Command ReviewControlCommand(Command current_command, State current_state) final;
    void BeforeFirstTick(BT::Blackboard&) final;
    ClosureConduct OnResult(bool success) final;

    rclcpp::Subscription<ContingencyEventMsg>::SharedPtr sub_contingency_event_ptr_;
    bool critical_event_detected_{false};
};

ContingencyManagerExecutor::ContingencyManagerExecutor(const rclcpp::NodeOptions& options)
    : BTExecutor{"contingency_manager", options, 5555}
{
    sub_contingency_event_ptr_ = node()->create_subscription<ContingencyEventMsg>(
        CONTINGENCY_EVENT_TOPIC_NAME,
        10,
        [this](std::unique_ptr<ContingencyEventMsg> msg) {
            // Publish event information to tree
            global_blackboard()->set<uint8_t>(KEY_EVENT_ID, msg->event_id);
            global_blackboard()->set<std::string>(KEY_EVENT_NAME, to_string(*msg));

            // Set flag to wake up the contingency manager if a critical event has been detected
            critical_event_detected_ = false;
            if (msg->event_id == ContingencyEventMsg::NO_EVENT) return;
            if (msg->event_id == ContingencyEventMsg::EVENT_UNDEFINED) {
                RCLCPP_WARN(node()->get_logger(), "Received an undefined contingency event");
                return;
            }
            critical_event_detected_ = true;
        });
}

void ContingencyManagerExecutor::SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr,
                                                          BT::BehaviorTreeFactory& factory)
{
    uas_behavior::RegisterNodePlugins(
        factory,
        node_ptr,
        uas_behavior::get_config_filepath("contingency_manager", "contingency_manager_bt_node_config"));

    // Enums (don't rely on magic enums for error safety)
    RegisterContingencyEventEnum(factory);
}

ContingencyManagerExecutor::Command ContingencyManagerExecutor::ReviewControlCommand(Command current_command,
                                                                                     State current_state)
{
    // Leave executor in IDLE until it needs to wake up
    if (!critical_event_detected_ && current_state == State::IDLE && current_command == Command::RUN)
        return Command::PAUSE;

    return current_command;
}

void ContingencyManagerExecutor::BeforeFirstTick(BT::Blackboard& global_blackboard)
{
    RCLCPP_INFO(node()->get_logger(),
                "Contingency Manager wakes up due to event %s",
                global_blackboard.get<std::string>(KEY_EVENT_NAME).c_str());
}

ContingencyManagerExecutor::ClosureConduct ContingencyManagerExecutor::OnResult(bool success)
{
    if (success) return ClosureConduct::RESTART;
    return ClosureConduct::ABORT;
}

}  // namespace contingency_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(contingency_manager::ContingencyManagerExecutor);
