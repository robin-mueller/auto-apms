#include <yaml-cpp/yaml.h>

#include <behaviortree_ros2/plugins.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <chrono>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <px4_behavior/factory.hpp>
#include <px4_behavior/get_resource.hpp>

namespace px4_behavior {

RegistrationStatus RegisterNodePlugins(BT::BehaviorTreeFactory& factory,
                                       const rclcpp::Node::SharedPtr& node,
                                       const std::filesystem::path& config_yaml,
                                       const std::optional<std::filesystem::path>& extra_plugin_directory)
{
    auto create_node_params = [node](const std::string& default_port_value = "") {
        BT::RosNodeParams params;
        params.nh = node;
        params.default_port_value = default_port_value;
        params.server_timeout = std::chrono::milliseconds(1500);
        params.wait_for_server_timeout = std::chrono::seconds(3);
        return params;
    };

    YAML::Node root;
    try {
        root = YAML::LoadFile(config_yaml);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(),
                     "RegisterNodePlugins: Error loading config file '%s' - %s",
                     config_yaml.c_str(),
                     e.what());
        return RegistrationStatus::MISSING_CONFIG;
    }

    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it) {
        const auto package_name = it->first.as<std::string>();
        const auto plugins = root[package_name];
        for (YAML::const_iterator it = plugins.begin(); it != plugins.end(); ++it) {
            const auto plugin_target_name = it->first.as<std::string>();
            const auto lib_file_name = "lib" + plugin_target_name + ".so";
            std::vector<std::filesystem::path> lib_dirs_priority_vec;

            // Check if the install already exists
            std::string install_plugin_dir;
            try {
                install_plugin_dir = get_bt_plugin_directory(package_name).string();
            } catch (const std::exception& e) {
                // Do nothing
            }

            // Use the install filepath unless the lib file is found in the extra plugin directory
            if (!install_plugin_dir.empty()) { lib_dirs_priority_vec.push_back(install_plugin_dir); }
            if (extra_plugin_directory.has_value()) { lib_dirs_priority_vec.push_back(extra_plugin_directory.value()); }
            std::string lib_filepath;
            for (const auto& dir : lib_dirs_priority_vec) {
                const auto check_path = dir / lib_file_name;
                if (std::filesystem::exists(check_path)) {
                    lib_filepath = check_path.string();
                    break;
                }
            }

            // Check if library file exists
            if (lib_filepath.empty()) {
                RCLCPP_ERROR(node->get_logger(),
                             "RegisterNodePlugins: Cannot load library '%s' because it exists neither in install '%s' "
                             "nor in extra '%s'",
                             lib_file_name.c_str(),
                             install_plugin_dir.c_str(),
                             extra_plugin_directory.value_or("").c_str());
                return RegistrationStatus::MISSING_PLUGIN_LIB;
            }

            try {
                if (it->second.IsNull()) {
                    // Standard BT plugin
                    RCLCPP_DEBUG(node->get_logger(),
                                 "RegisterNodePlugins: Loading standard BT node plugin library '%s'",
                                 lib_file_name.c_str());

                    // The default implementation doesn't throw currently, so we do it ourselves
                    BT::SharedLibrary loader;
                    loader.load(lib_filepath);
                    typedef void (*Func)(BT::BehaviorTreeFactory&);
                    Func func = (Func)loader.getSymbol(BT::PLUGIN_SYMBOL);
                    func(factory);
                }
                else {
                    // BT ROS2 plugin
                    RCLCPP_DEBUG(node->get_logger(),
                                 "RegisterNodePlugins: Loading ROS2 BT node plugin library '%s'",
                                 lib_file_name.c_str());

                    const auto port_value = it->second.as<std::string>();
                    RegisterRosNode(factory, lib_filepath, create_node_params(port_value));
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node->get_logger(),
                             "RegisterNodePlugins: Error loading library '%s' - %s",
                             lib_filepath.c_str(),
                             e.what());
                return RegistrationStatus::MISSING_PLUGIN_LIB;
            }
        }
    }
    return RegistrationStatus::SUCCESS;
}

}  // namespace px4_behavior
