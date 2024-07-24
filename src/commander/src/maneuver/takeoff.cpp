#include <commander/maneuver/flight_mode_executor.hpp>
#include <commander_interfaces/action/takeoff.hpp>

namespace commander {

class TakeoffManeuver : public FlightModeExecutor<commander_interfaces::action::Takeoff>
{
   public:
    explicit TakeoffManeuver(const rclcpp::NodeOptions& options)
        : FlightModeExecutor{commander::TAKEOFF_MANEUVER_NAME, options, FlightMode::Takeoff}
    {}

   private:
    bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr)
    {
        return client.Takeoff(goal_ptr->altitude_amsl_m, goal_ptr->heading_rad);
    }
};

}  // namespace commander


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(commander::TakeoffManeuver)
