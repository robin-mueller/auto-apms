#include "px4_behavior/bt_factory.hpp"

#include <optional>
#include <regex>

#include "class_loader/class_loader.hpp"
#include "px4_behavior/bt_node_factory.hpp"
#include "px4_behavior/get_resource.hpp"
#include "rcpputils/split.hpp"
#include "yaml-cpp/yaml.h"

namespace px4_behavior {

BT::RosNodeParams CreateRosNodeParams(const rclcpp::Node::SharedPtr& node,
                                      const std::string& default_port_value,
                                      const std::chrono::milliseconds& request_timeout,
                                      const std::chrono::milliseconds& wait_timeout)
{
    BT::RosNodeParams params;
    params.nh = node;
    params.default_port_value = default_port_value;
    params.server_timeout = request_timeout;
    params.wait_for_server_timeout = wait_timeout;
    return params;
}

RegistrationStatus RegisterBTNodePlugins(BT::BehaviorTreeFactory& factory,
                                         const rclcpp::Node::SharedPtr& node,
                                         const std::filesystem::path& config_yaml,
                                         const std::vector<std::string>& build_infos)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(config_yaml);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(),
                     "RegisterBTNodePlugins: Error loading config file '%s' - %s",
                     config_yaml.c_str(),
                     e.what());
        return RegistrationStatus::MISSING_CONFIG;
    }

    std::map<std::string, std::unique_ptr<class_loader::ClassLoader>> loaders;

    // Retrieve plugin library paths from build info of the current package (preferred)
    std::map<std::string, std::string> plugin_library_paths;
    for (const auto& build_info : build_infos) {
        std::vector<std::string> parts = rcpputils::split(build_info, '@');
        if (parts.size() != 2) { throw std::runtime_error("Invalid build info entry: " + build_info); }
        const std::string& classname = parts[0];
        const std::string& path = parts[1];
        plugin_library_paths[classname] = path;
    }

    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it) {
        const auto package_name = it->first.as<std::string>();
        const auto plugins_node = root[package_name];

        // Add plugin library paths from ament_index resources if not already populated
        for (const auto& resource : GetBTNodePluginResources(package_name)) {
            if (plugin_library_paths.find(resource.classname) == plugin_library_paths.end()) {
                plugin_library_paths[resource.classname] = resource.library_path;
            }
        }

        for (YAML::const_iterator it = plugins_node.begin(); it != plugins_node.end(); ++it) {
            const std::string node_plugin_key = it->first.as<std::string>();

            std::vector<std::string> matching_classnames{};
            for (const auto& r : plugin_library_paths) {
                // Allow node_plugin_key to associate with the classname without explicitly stating the full namespace
                if (std::smatch match;
                    std::regex_search(r.first, match, std::regex("(?:^|::)" + node_plugin_key + "$"))) {
                    matching_classnames.push_back(r.first);
                }
            }
            if (matching_classnames.empty()) {
                // Check fallback_plugin_paths if no matching resources are available
                RCLCPP_ERROR(
                    node->get_logger(),
                    "RegisterBTNodePlugins: Couldn't find a behavior tree node plugin with name '%s' in package '%s'",
                    node_plugin_key.c_str(),
                    package_name.c_str());
                return RegistrationStatus::MISSING_PLUGIN;
            }
            if (matching_classnames.size() > 1) {
                RCLCPP_ERROR(
                    node->get_logger(),
                    "RegisterBTNodePlugins: Cannot load plugin '%s' because multiple matching class names have "
                    "been found: %s\nIntroduce a unique namespace to the plugin key to resolve this issue.",
                    node_plugin_key.c_str(),
                    std::accumulate(matching_classnames.begin(),
                                    matching_classnames.end(),
                                    std::string(""),
                                    [](std::string& x, std::string& y) { return x.empty() ? y : x + ";" + y; })
                        .c_str());
                return RegistrationStatus::MISSING_PLUGIN;
            }

            const std::string& plugin_classname = matching_classnames[0];
            const std::string& plugin_library_path = plugin_library_paths[plugin_classname];

            if (loaders.find(plugin_library_path) == loaders.end()) {
                try {
                    loaders[plugin_library_path] = std::make_unique<class_loader::ClassLoader>(plugin_library_path);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node->get_logger(),
                                 "RegisterBTNodePlugins: Failed to load library %s - %s",
                                 plugin_library_path.c_str(),
                                 e.what());
                    return RegistrationStatus::MISSING_PLUGIN;
                } catch (...) {
                    RCLCPP_ERROR(node->get_logger(),
                                 "RegisterBTNodePlugins: Failed to load library %s",
                                 plugin_library_path.c_str());
                    return RegistrationStatus::MISSING_PLUGIN;
                }
            }
            class_loader::ClassLoader* loader = loaders[plugin_library_path].get();

            // Look if the class we search for is actually present in the library.
            const std::string factory_classname = "px4_behavior::BTNodeFactoryTemplate<" + plugin_classname + ">";
            if (!loader->isClassAvailable<BTNodeFactory>(factory_classname)) {
                RCLCPP_ERROR(node->get_logger(),
                             "RegisterBTNodePlugins: Class '%s' is not available in %s. You most likely misspelled the "
                             "class names of the corresponding behavior tree node in CMake when registering using "
                             "px4_behavior_register_plugins()",
                             factory_classname.c_str(),
                             loader->getLibraryPath().c_str());
                return RegistrationStatus::MISSING_PLUGIN;
            }

            auto plugin_instance = loader->createInstance<px4_behavior::BTNodeFactory>(factory_classname);

            RCLCPP_DEBUG(node->get_logger(),
                         "RegisterBTNodePlugins: Register behavior tree node plugin '%s (%s)' from library %s",
                         node_plugin_key.c_str(),
                         plugin_classname.c_str(),
                         plugin_library_path.c_str());

            plugin_instance->RegisterForBehaviorTreeFactory(
                factory,
                node_plugin_key,
                CreateRosNodeParams(node, it->second.IsNull() ? "" : it->second.as<std::string>()));
        }
    }
    return RegistrationStatus::SUCCESS;
}

}  // namespace px4_behavior
