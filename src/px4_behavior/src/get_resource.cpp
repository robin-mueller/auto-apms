#include "px4_behavior/get_resource.hpp"

#include <fstream>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "rcpputils/split.hpp"

namespace px4_behavior {

std::vector<BTNodePluginResource> GetBTNodePluginResources(const std::string& package_name)
{
    std::string content;
    std::string base_path;
    std::vector<BTNodePluginResource> resources{};
    if (ament_index_cpp::get_resource(_PX4_BEHAVIOR_BT_NODE_PLUGINS_RESOURCE_TYPE_NAME,
                                      package_name,
                                      content,
                                      &base_path)) {
        std::vector<std::string> lines = rcpputils::split(content, '\n', true);
        for (const auto& line : lines) {
            std::vector<std::string> parts = rcpputils::split(line, ';');
            if (parts.size() != 2) { throw std::runtime_error("Invalid resource entry"); }

            std::string library_path = parts[1];
            if (!std::filesystem::path(library_path).is_absolute()) { library_path = base_path + "/" + library_path; }
            resources.push_back({parts[0], library_path});
        }
    }
    return resources;
}

std::filesystem::path get_resource_directory(const std::string& package_name)
{
    return std::filesystem::path{ament_index_cpp::get_package_share_directory(package_name)} / "px4_behavior";
}

std::filesystem::path get_plugin_config_filepath(const std::string& package_name, const std::string& config_filename)
{
    auto filepath = get_resource_directory(package_name) / std::string(_PX4_BEHAVIOR_RESOURCES_PLUGIN_CONFIG_DIR_NAME) /
                    config_filename;

    if (!filepath.has_extension()) { filepath.replace_extension(".yaml"); }
    if (filepath.extension().compare(".yaml") != 0) {
        throw std::runtime_error("Argument config_filename '" + config_filename +
                                 "' has wrong extension. Must be '.yaml'");
    }
    if (!std::filesystem::exists(filepath)) {
        throw std::runtime_error("File '" + filepath.string() + "' doesn't exist");
    }
    return filepath;
}

std::filesystem::path get_behavior_tree_filepath(const std::string& package_name, const std::string& tree_filename)
{
    auto filepath = get_resource_directory(package_name) / std::string(_PX4_BEHAVIOR_RESOURCES_BEHAVIOR_TREE_DIR_NAME) /
                    tree_filename;

    if (!filepath.has_extension()) { filepath.replace_extension(".xml"); }
    if (filepath.extension().compare(".xml") != 0) {
        throw std::runtime_error("Argument tree_filename '" + tree_filename +
                                 "' has an invalid extension. Must be '.xml'");
    }
    if (!std::filesystem::exists(filepath)) {
        throw std::runtime_error("File '" + filepath.string() + "' doesn't exist");
    }
    return filepath;
}

std::string read_behavior_tree_filepath(const std::filesystem::path& tree_filepath)
{
    // Note that we have to use binary mode as we want to return a string matching the bytes of the file
    std::ifstream file(tree_filepath, std::ios::binary);
    if (!file.is_open()) { throw std::runtime_error("Couldn't open file '" + tree_filepath.string() + "'"); }

    // Read contents
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

}  // namespace px4_behavior
