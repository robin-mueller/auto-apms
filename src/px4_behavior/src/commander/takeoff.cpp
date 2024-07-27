#include <px4_behavior/commander/mode_executor.hpp>
#include <px4_behavior_interfaces/action/takeoff.hpp>

namespace px4_behavior {

class TakeoffManeuver : public ModeExecutor<px4_behavior_interfaces::action::Takeoff>
{
   public:
    explicit TakeoffManeuver(const rclcpp::NodeOptions& options)
        : ModeExecutor{px4_behavior::TAKEOFF_MANEUVER_NAME, options, FlightMode::Takeoff}
    {}

   private:
    bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr)
    {
        return client.Takeoff(goal_ptr->altitude_amsl_m, goal_ptr->heading_rad);
    }
};

}  // namespace px4_behavior


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(px4_behavior::TakeoffManeuver)
