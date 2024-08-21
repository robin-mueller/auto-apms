#include "px4_behavior/get_resource.hpp"

#include <fstream>
#include <functional>
#include <set>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"
#include "rcpputils/split.hpp"

namespace px4_behavior {

std::string ReadBehaviorTreeFile(const std::filesystem::path& tree_path)
{
    // Note that we have to use binary mode as we want to return a string matching the bytes of the file
    std::ifstream file(tree_path, std::ios::binary);
    if (!file.is_open()) { throw std::runtime_error("Couldn't open file '" + tree_path.string() + "'"); }

    // Read contents
    return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::vector<BTNodePluginResource> FetchBTNodePluginResources(const std::string& package_name)
{
    std::string content;
    std::string base_path;
    std::vector<BTNodePluginResource> resources;
    if (ament_index_cpp::get_resource(_PX4_BEHAVIOR_BT_NODE_PLUGINS__RESOURCE_TYPE_NAME,
                                      package_name,
                                      content,
                                      &base_path)) {
        std::vector<std::string> lines = rcpputils::split(content, '\n', true);
        for (const auto& line : lines) {
            std::vector<std::string> parts = rcpputils::split(line, '|');
            if (parts.size() != 2) { throw std::runtime_error("Invalid resource entry"); }

            const std::string& classname = parts[0];
            std::string library_path = base_path + "/" + parts[1];
            resources.push_back({classname, library_path});
        }
    }
    return resources;
}

std::vector<BehaviorTreeResource> FetchBehaviorTreeResources(const std::string& package_name)
{
    std::string content;
    std::string base_path;
    std::vector<BehaviorTreeResource> resources;
    if (ament_index_cpp::get_resource(_PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_TYPE_NAME,
                                      package_name,
                                      content,
                                      &base_path)) {
        std::vector<std::string> lines = rcpputils::split(content, '\n', true);
        auto make_absolute_path = [base_path](const std::string& s) { return base_path + "/" + s; };
        for (const auto& line : lines) {
            std::vector<std::string> parts = rcpputils::split(line, '|');
            if (parts.size() != 4) { throw std::runtime_error("Invalid resource entry"); }

            const std::string& filename = parts[0];
            std::string tree_path = make_absolute_path(parts[1]);
            std::vector<std::string> plugin_config_paths_vec = rcpputils::split(parts[2], ';');
            std::transform(plugin_config_paths_vec.begin(),
                           plugin_config_paths_vec.end(),
                           plugin_config_paths_vec.begin(),
                           make_absolute_path);

            std::vector<std::string> tree_ids_vec = rcpputils::split(parts[3], ';');
            resources.push_back({filename,
                                 tree_path,
                                 {plugin_config_paths_vec.begin(), plugin_config_paths_vec.end()},
                                 {tree_ids_vec.begin(), tree_ids_vec.end()}});
        }
    }
    return resources;
}

std::optional<BehaviorTreeResource> FetchBehaviorTreeResource(std::optional<const std::string> tree_file_name,
                                                              std::optional<const std::string> tree_id,
                                                              std::optional<const std::string> package_name)
{
    std::vector<std::string> search_packages;
    if (package_name.has_value()) { search_packages.push_back(package_name.value()); }
    else {
        for (const auto& r : ament_index_cpp::get_resources(_PX4_BEHAVIOR_BEHAVIOR_TREE__RESOURCE_TYPE_NAME))
            search_packages.push_back(r.first);
    }

    std::vector<BehaviorTreeResource> matching_resources;
    std::function<bool(const BehaviorTreeResource&)> is_resource_matching;
    if (tree_file_name.has_value()) {
        is_resource_matching = [fn = tree_file_name.value()](const BehaviorTreeResource& r) {
            return r.tree_file_name == fn;
        };
    }
    else if (tree_id.has_value()) {
        is_resource_matching = [tid = tree_id.value()](const BehaviorTreeResource& r) {
            return r.tree_ids.find(tid) != r.tree_ids.end();
        };
    }
    else {
        throw std::runtime_error("Either of arguments tree_file_name or tree_id must be specified");
    }

    for (const auto& package_name : search_packages) {
        const auto resources = FetchBehaviorTreeResources(package_name);
        if (resources.empty()) continue;
        for (const auto& r : resources) {
            if (is_resource_matching(r)) matching_resources.push_back(r);
        }
    }

    if (matching_resources.empty()) return {};
    if (matching_resources.size() > 1) throw std::runtime_error("There are multiple matching resources");
    return matching_resources[0];
}

}  // namespace px4_behavior
