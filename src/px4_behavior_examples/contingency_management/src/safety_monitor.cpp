#include <definitions.hpp>
#include <px4_behavior/bt_executor.hpp>
#include <px4_behavior/factory.hpp>
#include <px4_behavior_examples/msg/contingency_event.hpp>
#include <px4_behavior_examples/msg/force_contingency.hpp>
#include <px4_behavior_examples/msg/landing_site_status.hpp>
#include <px4_behavior_examples/msg/system_state.hpp>

#define KEY_EVENT_ID "event_id"

using namespace px4_behavior;
using ForceContingencyMsg = px4_behavior_examples::msg::ForceContingency;
using SystemStateMsg = px4_behavior_examples::msg::SystemState;
using LandingSiteStatusMsg = px4_behavior_examples::msg::LandingSiteStatus;
using ContingencyEventMsg = px4_behavior_examples::msg::ContingencyEvent;

class SafetyMonitorExecutor : public BTExecutor
{
   public:
    SafetyMonitorExecutor(const rclcpp::NodeOptions& options);

   private:
    void SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory) final;
    ClosureConduct OnResult(bool success) final;

    rclcpp::Subscription<ForceContingencyMsg>::SharedPtr sub_force_contingency_ptr_;
    rclcpp::Publisher<SystemStateMsg>::SharedPtr pub_system_state_ptr_;
    rclcpp::Publisher<LandingSiteStatusMsg>::SharedPtr pub_landing_site_status_ptr_;
    rclcpp::Publisher<ContingencyEventMsg>::SharedPtr pub_contingency_event_ptr_;

    rclcpp::TimerBase::SharedPtr publish_timer_ptr_;
    float battery_level_percent_{100};
    std::map<size_t, uint8_t> landing_site_status_map_;
};

SafetyMonitorExecutor::SafetyMonitorExecutor(const rclcpp::NodeOptions& options)
    : BTExecutor{"safety_monitor", options, 4444}
{
    // Landing site status map init
    landing_site_status_map_[0] = LandingSiteStatusMsg::STATUS_CLEAR_FOR_LANDING;
    landing_site_status_map_[1] = LandingSiteStatusMsg::STATUS_CLEAR_FOR_LANDING;

    sub_force_contingency_ptr_ = node()->create_subscription<ForceContingencyMsg>(
        FORCE_CONTINGENCY_TOPIC_NAME,
        10,
        [this](std::unique_ptr<ForceContingencyMsg> msg) {
            switch (msg->force_action_id) {
                case ForceContingencyMsg::FORCE_BATTERY_OK:
                    this->battery_level_percent_ = 100;
                    return;
                case ForceContingencyMsg::FORCE_BATTERY_CRITICAL:
                    this->battery_level_percent_ = 0;
                    return;
                case ForceContingencyMsg::FORCE_LANDING_SITE_CLEAR:
                    landing_site_status_map_[msg->landing_site_id] = LandingSiteStatusMsg::STATUS_CLEAR_FOR_LANDING;
                    return;
                case ForceContingencyMsg::FORCE_LANDING_SITE_TEMPORARILY_BLOCKED:
                    landing_site_status_map_[msg->landing_site_id] = LandingSiteStatusMsg::STATUS_TEMPORARILY_BLOCKED;
                    return;
                case ForceContingencyMsg::FORCE_LANDING_SITE_PERMANENTLY_BLOCKED:
                    landing_site_status_map_[msg->landing_site_id] = LandingSiteStatusMsg::STATUS_PERMANENTLY_BLOCKED;
                    return;
            }
        });

    pub_system_state_ptr_ = node()->create_publisher<SystemStateMsg>(SYSTEM_STATE_TOPIC_NAME, 10);
    pub_landing_site_status_ptr_ = node()->create_publisher<LandingSiteStatusMsg>(LANDING_SITE_STATUS_TOPIC_NAME, 10);
    pub_contingency_event_ptr_ = node()->create_publisher<ContingencyEventMsg>(CONTINGENCY_EVENT_TOPIC_NAME, 10);

    publish_timer_ptr_ = node()->create_wall_timer(std::chrono::milliseconds(10), [this]() {
        SystemStateMsg state_msg;
        state_msg.battery_level_percent = battery_level_percent_;
        pub_system_state_ptr_->publish(state_msg);

        LandingSiteStatusMsg landing_site_status_msg;
        for (const auto& status : landing_site_status_map_) { landing_site_status_msg.status.push_back(status.second); }
        pub_landing_site_status_ptr_->publish(landing_site_status_msg);
    });
}

void SafetyMonitorExecutor::SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory)
{
    px4_behavior::RegisterNodePlugins(
        factory,
        node_ptr,
        px4_behavior::get_plugin_config_filepath("px4_behavior", "safety_monitor_bt_node_config"));

    // Enums (don't rely on magic enums for error safety)
    RegisterContingencyEventEnum(factory);
    factory.registerScriptingEnum("LANDING_SITE_TEMP_BLOCKED", LandingSiteStatusMsg::STATUS_TEMPORARILY_BLOCKED);
    factory.registerScriptingEnum("LANDING_SITE_PERM_BLOCKED", LandingSiteStatusMsg::STATUS_PERMANENTLY_BLOCKED);
}

SafetyMonitorExecutor::ClosureConduct SafetyMonitorExecutor::OnResult(bool success)
{
    ContingencyEventMsg event_msg;
    if (success) { event_msg.event_id = ContingencyEventMsg::NO_EVENT; }
    else {
        if (!global_blackboard()->get<uint8_t>(KEY_EVENT_ID, event_msg.event_id)) {
            event_msg.event_id = ContingencyEventMsg::EVENT_UNDEFINED;
            RCLCPP_ERROR(node()->get_logger(),
                         "Tree does not set global variable '%s', so contingency event is undefined",
                         KEY_EVENT_ID);
        }
    }
    pub_contingency_event_ptr_->publish(event_msg);
    return ClosureConduct::RESTART;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SafetyMonitorExecutor);
