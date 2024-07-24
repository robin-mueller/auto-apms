#include <uas_behavior/bt_executor.hpp>
#include <uas_behavior/factory.hpp>

namespace contingency_manager {

class FlightOrchestratorExecutor : public uas_behavior::BTExecutor
{
   public:
    FlightOrchestratorExecutor(const rclcpp::NodeOptions& options) : BTExecutor{"flight_orchestrator", options, 7777} {}

   private:
    void SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory) final
    {
        uas_behavior::RegisterNodePlugins(
            factory,
            node_ptr,
            uas_behavior::get_config_filepath("contingency_manager", "orchestrator_bt_node_config"));
    }
};

}  // namespace contingency_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(contingency_manager::FlightOrchestratorExecutor);
