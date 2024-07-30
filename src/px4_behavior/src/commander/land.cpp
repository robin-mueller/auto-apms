#include <px4_behavior/commander/mode_executor.hpp>
#include <px4_behavior_interfaces/action/land.hpp>

namespace px4_behavior {

class LandTask : public ModeExecutor<px4_behavior_interfaces::action::Land>
{
   public:
    explicit LandTask(const rclcpp::NodeOptions& options)
        : ModeExecutor{px4_behavior::LAND_TASK_NAME, options, FlightMode::Land}
    {}

   private:
    bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr)
    {
        (void)goal_ptr;
        return client.Land();
    }
};

}  // namespace px4_behavior

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(px4_behavior::LandTask)
