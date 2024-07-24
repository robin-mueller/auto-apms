#include <commander/maneuver/flight_mode_executor.hpp>
#include <commander_interfaces/action/mission.hpp>

namespace commander {

class MissionManeuver : public FlightModeExecutor<commander_interfaces::action::Mission>
{
   public:
    explicit MissionManeuver(const rclcpp::NodeOptions& options)
        : FlightModeExecutor{commander::MISSION_MANEUVER_NAME, options, FlightMode::Mission}
    {}

   private:
    bool SendActivationCommand(const VehicleCommandClient& client, std::shared_ptr<const Goal> goal_ptr) final
    {
        if (goal_ptr->do_restart) { return client.StartMission(); }
        return client.SyncActivateFlightMode(FlightMode::Mission);
    }
};

}  // namespace commander

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(commander::MissionManeuver)
