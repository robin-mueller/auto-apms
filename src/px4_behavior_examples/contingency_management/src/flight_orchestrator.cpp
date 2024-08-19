#include <px4_behavior/bt_executor.hpp>
#include <px4_behavior/bt_factory.hpp>

using namespace px4_behavior;

namespace px4_behavior::ops_engine {

class FlightOrchestratorExecutor : public BTExecutor
{
   public:
    FlightOrchestratorExecutor(const rclcpp::NodeOptions& options) : BTExecutor{"flight_orchestrator", options, 7777} {}

   private:
    void SetupBehaviorTreeFactory(rclcpp::Node::SharedPtr node_ptr, BT::BehaviorTreeFactory& factory) final
    {
        RegisterBTNodePlugins(factory,
                            node_ptr,
                            get_plugin_config_filepath("px4_behavior", "orchestrator_bt_node_config"));
    }
};

}  // namespace px4_behavior::ops_engine

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(px4_behavior::ops_engine::FlightOrchestratorExecutor);
