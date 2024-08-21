#pragma once

#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace px4_behavior {

struct BTNodePluginResource
{
    std::string classname;
    std::string library_path;
};

struct BehaviorTreeResource
{
    std::string tree_file_name;
    std::string tree_path;
    std::set<std::string> plugin_config_paths;
    std::set<std::string> tree_ids;
};

/**
 * \brief Read a trees file.
 *
 * Read the data of the file \p tree_path and return the raw string.
 *
 * \param tree_path Path to the file containing the behavior tree's definition
 * \return XML string of the behavior tree file.
 * \throw std::runtime_error if failed to read file.
 */
std::string ReadBehaviorTreeFile(const std::filesystem::path& tree_path);

std::vector<BTNodePluginResource> FetchBTNodePluginResources(const std::string& package_name);

std::vector<BehaviorTreeResource> FetchBehaviorTreeResources(const std::string& package_name);

std::optional<BehaviorTreeResource> FetchBehaviorTreeResource(
    std::optional<const std::string> tree_file_name = std::nullopt,
    std::optional<const std::string> tree_id = std::nullopt,
    std::optional<const std::string> package_name = std::nullopt);

}  // namespace px4_behavior