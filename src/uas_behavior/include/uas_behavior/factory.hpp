#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <uas_behavior/get_resource.hpp>

namespace uas_behavior {

enum class RegistrationStatus : uint8_t {
    SUCCESS,
    MISSING_CONFIG,
    MISSING_PLUGIN_LIB,
};

RegistrationStatus RegisterNodePlugins(
    BT::BehaviorTreeFactory& factory,
    const rclcpp::Node::SharedPtr& node,
    const std::filesystem::path& config_yaml,
    const std::optional<std::filesystem::path>& extra_plugin_directory = std::nullopt);

}  // namespace uas_behavior
