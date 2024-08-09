#pragma once

#include <filesystem>
#include <string>

namespace px4_behavior {
    
/**
 * \brief Get the px4_behavior resource subdirectory located in the share directory of an installed package.
 *
 * \param package_name Name of the package
 * \return Absolute path to the px4_behavior resource subdirectory.
 */
std::filesystem::path get_shared_resource_directory(const std::string& package_name);

/**
 * \brief Get the directory where the implemented behavior tree plugins are located.
 *
 * The plugin directory is installed under `lib` in the package install directory which has the following structure:
 * ```
 *      -- package_name
 *         |-- include
 *         |-- lib
 *         |    -- px4_behavior
 *         |        -- lib<plugin_target_name>.so
 *         |        -- ... # More plugins named like above
 *         |    -- ... # Other libraries
 *         |-- share
 * ```
 *
 * \param package_name Name of the package
 * \return Absolute path to the plugin directory.
 */
std::filesystem::path get_bt_plugin_directory(const std::string& package_name);

/**
 * \brief Get the filepath of a config file in the share directory of a installed package.
 *
 * The referenced package is expected to have registered px4_behavior resources via CMake.
 *
 * \param package_name Name of the package
 * \param config_filename Name of the node configuration file (extension can be omitted)
 * \return Absolute path to the configuration file.
 * \throw std::runtime_error if config file cannot be found.
 */
std::filesystem::path get_config_filepath(const std::string& package_name, const std::string& config_filename);

/**
 * \brief Get the filepath of a trees file in the share directory of an installed package.
 *
 * The referenced package is expected to have registered px4_behavior resources via CMake.
 *
 * \param package_name Name of the package
 * \param trees_filename Name of the trees file (extension can be omitted)
 * \return Absolute path to the trees file.
 * \throw std::runtime_error if trees file cannot be found.
 */
std::filesystem::path get_trees_filepath(const std::string& package_name, const std::string& trees_filename);

/**
 * \brief Read a trees file.
 * 
 * Read the data of the file \p trees_filepath and return the raw string.
 * 
 * \param trees_filepath Path to the file containing data of trees
 * \return XML string of the trees.
 * \throw std::runtime_error if failed to read file.
*/
std::string read_trees_filepath(const std::filesystem::path trees_filepath);

}  // namespace px4_behavior