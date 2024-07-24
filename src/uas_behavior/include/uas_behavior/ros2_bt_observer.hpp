#pragma once

#include <behaviortree_cpp/loggers/abstract_logger.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace uas_behavior {

class BTStateObserver : public BT::StatusChangeLogger
{
   public:
    BTStateObserver(const BT::Tree& tree,
                    const rclcpp::Logger& node_logger,
                    std::chrono::seconds max_logging_interval = std::chrono::seconds(1));

    virtual void flush() override;

    void set_state_change_logging(bool active) { state_change_logging_ = active; };

    const std::vector<std::string>& running_action_history() { return running_action_history_; }
    const std::string& last_running_action_name() { return last_running_action_name_; }

   private:
    // Creates a bitmask that uniquely identifies a node's state change
    uint16_t CreateStateChangeBitmask(BT::NodeStatus prev_status, BT::NodeStatus curr_status);

    virtual void callback(BT::Duration timestamp,
                          const BT::TreeNode& node,
                          BT::NodeStatus prev_status,
                          BT::NodeStatus status) override;

    const rclcpp::Logger logger_;
    const std::string root_tree_id_;
    const std::chrono::seconds max_logging_interval_;
    bool state_change_logging_{false};
    std::vector<std::string> running_action_history_;
    std::string last_running_action_name_;
    std::map<std::pair<uint16_t, uint16_t>, BT::Duration> last_log_map_;
};

}  // namespace uas_behavior
