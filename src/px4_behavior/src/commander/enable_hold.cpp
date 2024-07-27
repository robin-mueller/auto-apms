#include <px4_behavior/commander/mode_executor.hpp>
#include <px4_behavior_interfaces/action/enable_hold.hpp>

namespace px4_behavior {

class EnableHoldManeuver : public ModeExecutor<px4_behavior_interfaces::action::EnableHold>
{
   public:
    explicit EnableHoldManeuver(const rclcpp::NodeOptions& options)
        : ModeExecutor{
              px4_behavior::ENABLE_HOLD_MANEUVER_NAME, options, FlightMode::Hold, false}
    {}

   private:
    bool IsCompleted(std::shared_ptr<const Goal> goal_ptr, const px4_msgs::msg::VehicleStatus& vehicle_status) final
    {
        (void)goal_ptr;
        (void)vehicle_status;
        // For HOLD mode there is no completion signal. Once activated it is considered complete.
        return true;
    }
};

}  // namespace px4_behavior

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(px4_behavior::EnableHoldManeuver)
