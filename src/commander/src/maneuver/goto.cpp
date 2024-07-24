#include <Eigen/Core>
#include <commander/maneuver/flight_mode_executor.hpp>
#include <commander_interfaces/action/go_to.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/utils/geodesic.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace commander {

using GoToActionType = commander_interfaces::action::GoTo;

class GoToMode : public PositionAwareMode<GoToActionType>
{
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint_ptr_;
    Eigen::Vector3d position_target_f_glob_;
    float heading_target_rad_;

   public:
    GoToMode(rclcpp::Node& node,
             const px4_ros2::ModeBase::Settings& settings,
             const std::string& topic_namespace_prefix,
             std::shared_ptr<ActionContextType> action_context_ptr)
        : PositionAwareMode{node, settings, topic_namespace_prefix, action_context_ptr}
    {
        goto_setpoint_ptr_ = std::make_shared<px4_ros2::GotoGlobalSetpointType>(*this);
    }

   private:
    void OnActivateWithGoal(std::shared_ptr<const Goal> goal_ptr) final
    {
        position_target_f_glob_ = Eigen::Vector3d(goal_ptr->lat, goal_ptr->lon, goal_ptr->alt);

        // Heading target has to be in rad clockwise from north
        if (goal_ptr->head_towards_destination) {
            // Heading target needs to be in interval [-pi, pi] (from north)
            heading_target_rad_ =
                px4_ros2::headingToGlobalPosition(vehicle_global_position_ptr_->position(), position_target_f_glob_);
        }
        else {
            heading_target_rad_ = goal_ptr->heading_clockwise_from_north_rad;
        }

        auto double_to_string = [](double num, int decimals) {
            std::stringstream stream;
            stream << std::fixed << std::setprecision(decimals) << num;
            return stream.str();
        };

        RCLCPP_DEBUG(node().get_logger(),
                     "GoTo target: Position: [%s°, %s°, %sm] - Heading: %f",
                     double_to_string(position_target_f_glob_.x(), 10).c_str(),
                     double_to_string(position_target_f_glob_.y(), 10).c_str(),
                     double_to_string(position_target_f_glob_.z(), 2).c_str(),
                     heading_target_rad_);
    }
    void UpdateSetpointWithGoal(float dt_s,
                                std::shared_ptr<const Goal> goal_ptr,
                                std::shared_ptr<Feedback> feedback_ptr,
                                std::shared_ptr<Result> result_ptr) final
    {
        (void)goal_ptr;
        (void)dt_s;
        (void)result_ptr;

        goto_setpoint_ptr_->update(position_target_f_glob_, heading_target_rad_);

        // Remaining distance to target
        Eigen::Vector3d distance_f_ned =
            px4_ros2::vectorToGlobalPosition(vehicle_global_position_ptr_->position(), position_target_f_glob_)
                .cast<double>();
        tf2::convert(distance_f_ned, feedback_ptr->remaining_distance_f_ned);

        // Estimated remaining time. Use only velocity in direction of target for estimation.
        feedback_ptr->remaining_time_s =
            distance_f_ned.norm() /
            (vehicle_local_position_ptr_->velocityNed().cast<double>().dot(distance_f_ned.normalized()));

        if (IsPositionReached(position_target_f_glob_)) { completed(px4_ros2::Result::Success); }
    }
};

class GoToManeuver : public ExternalFlightModeExecutor<GoToActionType, GoToMode>
{
   public:
    explicit GoToManeuver(const rclcpp::NodeOptions& options)
        : ExternalFlightModeExecutor{commander::GO_TO_MANEUVER_NAME, options}
    {}
};

}  // namespace commander

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(commander::GoToManeuver)
