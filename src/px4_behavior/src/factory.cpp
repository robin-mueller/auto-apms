#include <yaml-cpp/yaml.h>

#include <behaviortree_ros2/plugins.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <chrono>
#include <optional>
#include <px4_behavior/factory.hpp>
#include <px4_behavior/get_resource.hpp>
#include <rclcpp/rclcpp.hpp>

namespace px4_behavior {

RegistrationStatus RegisterNodePlugins(BT::BehaviorTreeFactory& factory,
                                       const rclcpp::Node::SharedPtr& node,
                                       const std::filesystem::path& config_yaml,
                                       const std::vector<std::filesystem::path>& extra_plugin_paths)
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
        const auto plugins_node = root[package_name];
        for (YAML::const_iterator it = plugins_node.begin(); it != plugins_node.end(); ++it) {
            const std::string plugin_target_name = it->first.as<std::string>();
            const std::string lib_file_name = "lib" + plugin_target_name + ".so";

            // Check extra_plugin_paths first before falling back to searching in the installation
            std::string lib_filepath;
            for (const auto &plugin_path : extra_plugin_paths) {
                if (plugin_path.filename().string() == lib_file_name) {
                    lib_filepath = plugin_path.string();
                    break;  // After a matching plugin path has been found, break immediately
                }
            }
            if (lib_filepath.empty()) {
                // Check if the install share dir exists
                std::filesystem::path install_plugin_dir;
                try {
                    install_plugin_dir = get_bt_plugin_directory(package_name).string();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node->get_logger(),
                                 "RegisterNodePlugins: Cannot load library '%s' because no matching plugin path was "
                                 "provided and package '%s' hasn't been installed yet",
                                 lib_file_name.c_str(),
                                 package_name.c_str());
                    return RegistrationStatus::MISSING_PLUGIN_LIB;
                }

                // Check if library file exists in share dir
                if (const auto p = install_plugin_dir / lib_file_name; std::filesystem::exists(p)) {
                    lib_filepath = p.string();
                }
                else {
                    RCLCPP_ERROR(
                        node->get_logger(),
                        "RegisterNodePlugins: Cannot load library '%s' because no matching plugin path was "
                                 "provided and it cannot be found in the share directory of package '%s': '%s'",
                        lib_file_name.c_str(),
                        package_name.c_str(),
                        install_plugin_dir.c_str());
                    return RegistrationStatus::MISSING_PLUGIN_LIB;
                }
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
