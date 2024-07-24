#include <px4_behavior/bt_executor.hpp>
#include <px4_behavior/factory.hpp>

namespace px4_behavior {

class FlightOrchestratorExecutor : public px4_behavior::BTExecutor
{
   public:
    FlightOrchestratorExecutor(const rclcpp::NodeOptions& options) : BTExecutor{"flight_orchestrator", options, 7777} {}

   private:
    void SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory) final
    {
        px4_behavior::RegisterNodePlugins(
            factory,
            node_ptr,
            px4_behavior::get_config_filepath("px4_behavior", "orchestrator_bt_node_config"));
    }
};

}  // namespace px4_behavior

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(px4_behavior::FlightOrchestratorExecutor);
