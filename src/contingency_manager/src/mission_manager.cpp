#include <contingency_manager/definitions.hpp>
#include <contingency_manager_interfaces/msg/landing_approach.hpp>
#include <uas_behavior/bt_executor.hpp>
#include <uas_behavior/factory.hpp>

#define KEY_ALTITUDE "altitude_amsl_m"
#define KEY_NEXT_LANDING_SITE_ID "next_landing_site_id"
#define KEY_IS_APPROACHING_LANDING "is_approaching_landing"

using LandingApproachMsg = contingency_manager_interfaces::msg::LandingApproach;

namespace contingency_manager {

class MissionManagerExecutor : public uas_behavior::BTExecutor
{
   public:
    MissionManagerExecutor(const rclcpp::NodeOptions& options);

   private:
    void SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory) final;
    void OnTreeCreated(BT::Blackboard& global_blackboard) final;

    BT::Blackboard::Ptr initial_bb_;
    rclcpp::Publisher<LandingApproachMsg>::SharedPtr pub_landing_approach_ptr_;
    rclcpp::TimerBase::SharedPtr publish_timer_ptr_;
};

MissionManagerExecutor::MissionManagerExecutor(const rclcpp::NodeOptions& options)
    : BTExecutor{"mission_manager", options, 6666}, initial_bb_{BT::Blackboard::create()}
{
    initial_bb_->set<double>(KEY_ALTITUDE, 190.0);
    initial_bb_->set<int>(KEY_NEXT_LANDING_SITE_ID, 0);
    initial_bb_->set<bool>(KEY_IS_APPROACHING_LANDING, false);
    initial_bb_->cloneInto(*global_blackboard());

    pub_landing_approach_ptr_ = node()->create_publisher<LandingApproachMsg>(LANDING_APPROCH_TOPIC_NAME, 10);
    publish_timer_ptr_ = node()->create_wall_timer(std::chrono::milliseconds(10), [this]() {
        LandingApproachMsg landing_approach_msg;
        landing_approach_msg.next_landing_site_id = global_blackboard()->get<int>(KEY_NEXT_LANDING_SITE_ID);
        landing_approach_msg.is_approaching = global_blackboard()->get<bool>(KEY_IS_APPROACHING_LANDING);
        pub_landing_approach_ptr_->publish(landing_approach_msg);
    });
}

void MissionManagerExecutor::SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr,
                                                      BT::BehaviorTreeFactory& factory)
{
    uas_behavior::RegisterNodePlugins(
        factory,
        node_ptr,
        uas_behavior::get_config_filepath("contingency_manager", "mission_bt_node_config"));
}

void MissionManagerExecutor::OnTreeCreated(BT::Blackboard& global_blackboard)
{
    initial_bb_->cloneInto(global_blackboard);  // Reset the global blackboard
}

}  // namespace contingency_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(contingency_manager::MissionManagerExecutor);
