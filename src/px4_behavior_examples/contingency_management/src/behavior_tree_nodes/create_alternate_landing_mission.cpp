// Copyright 2024 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <regex>

#include "Eigen/Geometry"
#include "px4_behavior/bt_ros2_node.hpp"
#include "px4_behavior/get_resource.hpp"
#include "px4_ros2/utils/geodesic.hpp"

#define INPUT_KEY_POS "pos_vec"
#define OUTPUT_KEY_DATA "xml_data"
#define OUTPUT_KEY_ID "tree_id"

using namespace BT;

namespace px4_behavior::ops_engine {

class CreateAlternateLandingMission : public SyncActionNode
{
   public:
    using SyncActionNode::SyncActionNode;

    static PortsList providedPorts()
    {
        return {InputPort<Eigen::Vector3d>(INPUT_KEY_POS,
                                           "Current global position (latitude [°], longitude [°], altitude AMSL [m])"),
                OutputPort<std::string>(OUTPUT_KEY_DATA,
                                        "{xml_data}",
                                        "String containing the XML data of the alternate landing mission tree"),
                OutputPort<std::string>(OUTPUT_KEY_ID,
                                        "{tree_id}",
                                        "The ID of the tree that acts as the entry point for the mission")};
    }

    NodeStatus tick() final
    {
        Eigen::Vector2d current_global_pos;
        double current_alt;
        if (auto any_locked = getLockedPortContent(INPUT_KEY_POS)) {
            if (any_locked->empty()) {
                throw std::runtime_error(std::string(name().c_str()) + " - Value at blackboard entry {" +
                                         INPUT_KEY_POS + "} is empty");
            }
            else if (Eigen::Vector3d* vec_ptr = any_locked->castPtr<Eigen::Vector3d>()) {
                current_global_pos = Eigen::Vector2d{vec_ptr->x(), vec_ptr->y()};
                current_alt = vec_ptr->z();
            }
            else {
                throw std::runtime_error(std::string(name().c_str()) + " - Failed to cast pointer {" + INPUT_KEY_POS +
                                         "}");
            }
        }
        else {
            throw std::runtime_error(std::string(name().c_str()) + " - getLockedPortContent() failed for argument " +
                                     INPUT_KEY_POS);
        }

        auto move_down = px4_ros2::addVectorToGlobalPosition(current_global_pos, {-5, 0});
        auto move_right = px4_ros2::addVectorToGlobalPosition(current_global_pos, {-5, 5});

        // Create waypoints depending on the current position
        std::vector<Eigen::Vector3d> waypoints{{move_down.x(), move_down.y(), current_alt},
                                               {move_right.x(), move_right.y(), current_alt - 2}};

        // Placeholder value map
        std::map<std::string, double> map;
        for (size_t i = 0; i < waypoints.size(); i++) {
            map["WAYPOINT" + std::to_string(i) + "_LAT"] = waypoints[i].x();
            map["WAYPOINT" + std::to_string(i) + "_LON"] = waypoints[i].y();
            map["WAYPOINT" + std::to_string(i) + "_ALT"] = waypoints[i].z();
        }

        auto double_to_string = [](double num, int decimals) {
            std::stringstream stream;
            stream << std::fixed << std::setprecision(decimals) << num;
            return stream.str();
        };

        // Read tree template and replace placeholders
        const std::string main_tree_id = "AlternateLandingMission";
        auto resource = FetchBehaviorTreeResource(std::nullopt, main_tree_id, "px4_behavior_examples");
        if (!resource.has_value()) { return NodeStatus::FAILURE; }
        auto tree = px4_behavior::ReadBehaviorTreeFile(resource.value().tree_path);

        // Search for pattern ${SOME_NAME} allowing letters, numbers, _ and -
        std::regex placeholder("\\$\\{([A-Za-z0-9_-]+)\\}");
        for (std::smatch match; std::regex_search(tree, match, placeholder);) {
            std::string full_match_str = match[0];
            std::string placeholder_name = match[1];

            // Replace the placeholder with an actual value or an empty string if not found in map
            tree.replace(match.position(),
                         full_match_str.length(),
                         map.find(placeholder_name) != map.end() ? double_to_string(map[placeholder_name], 14) : "");
        }

        setOutput(OUTPUT_KEY_DATA, tree);

        // Currently there is no easy way to read the tree id from the xml without creating it
        setOutput<std::string>(OUTPUT_KEY_ID, main_tree_id);
        return NodeStatus::SUCCESS;
    }
};

}  // namespace px4_behavior::ops_engine

#include "px4_behavior/register_behavior_tree_node_macro.hpp"
PX4_BEHAVIOR_REGISTER_BEHAVIOR_TREE_NODE(px4_behavior::ops_engine::CreateAlternateLandingMission)
