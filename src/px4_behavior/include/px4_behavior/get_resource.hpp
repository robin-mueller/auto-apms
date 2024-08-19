#pragma once

#include <filesystem>
#include <string>
#include <vector>
#include <utility>

namespace px4_behavior {

struct BTNodePluginResource {
    std::string classname;
    std::string library_path;
};

std::vector<BTNodePluginResource> GetBTNodePluginResources(const std::string& package_name);

/**
 * \brief Get the px4_behavior resource subdirectory located in the share directory of an installed package.
 *
 * \param package_name Name of the package
 * \return Absolute path to the px4_behavior resource subdirectory.
 */
std::filesystem::path get_resource_directory(const std::string& package_name);

/**
 * \brief Get the filepath of a config file in the share directory of a installed package.
 *
 * \param package_name Name of the package
 * \param config_filename Name of the node configuration file (extension can be omitted)
 * \return Absolute path to the configuration file.
 * \throw std::runtime_error if config file cannot be found.
 */
std::filesystem::path get_plugin_config_filepath(const std::string& package_name, const std::string& config_filename);

/**
 * \brief Get the filepath of a behavior tree file in the share directory of an installed package.
 *
 * \param package_name Name of the package
 * \param tree_filename Name of the behavior tree file (extension can be omitted)
 * \return Absolute path to the behavior tree file.
 * \throw std::runtime_error if behavior tree file cannot be found.
 */
std::filesystem::path get_behavior_tree_filepath(const std::string& package_name, const std::string& tree_filename);

/**
 * \brief Read a trees file.
 *
 * Read the data of the file \p trees_filepath and return the raw string.
 *
 * \param trees_filepath Path to the file containing the behavior tree's definition
 * \return XML string of the behavior tree file.
 * \throw std::runtime_error if failed to read file.
 */
std::string read_behavior_tree_filepath(const std::filesystem::path& trees_filepath);

}  // namespace px4_behavior