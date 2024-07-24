#include <ament_index_cpp/get_package_prefix.hpp>
#include <fstream>
#include <uas_behavior/get_resource.hpp>

namespace uas_behavior {

std::filesystem::path get_shared_resource_directory(const std::string& package_name)
{
    return std::filesystem::path{ament_index_cpp::get_package_prefix(package_name)} / "share" /
           std::string(UAS_BEHAVIOR_SHARE_DIR_NAME);
}

std::filesystem::path get_bt_plugin_directory(const std::string& package_name)
{
    return std::filesystem::path{ament_index_cpp::get_package_prefix(package_name)} / "lib" /
           std::string(UAS_BEHAVIOR_SHARE_DIR_NAME);
}

std::filesystem::path get_config_filepath(const std::string& package_name, const std::string& config_filename)
{
    auto filepath = get_shared_resource_directory(package_name) / std::string(UAS_BEHAVIOR_SHARE_SUBDIR_NAME_CONFIG) /
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

std::filesystem::path get_trees_filepath(const std::string& package_name, const std::string& tree_filename)
{
    auto filepath =
        get_shared_resource_directory(package_name) / std::string(UAS_BEHAVIOR_SHARE_SUBDIR_NAME_TREES) / tree_filename;

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

std::string read_trees_filepath(const std::filesystem::path trees_filepath)
{
    // Note that we have to use binary mode as we want to return a string matching the bytes of the file
    std::ifstream file(trees_filepath, std::ios::binary);
    if (!file.is_open()) { throw std::runtime_error("Couldn't open file '" + trees_filepath.string() + "'"); }

    // Read contents
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

}  // namespace uas_behavior
