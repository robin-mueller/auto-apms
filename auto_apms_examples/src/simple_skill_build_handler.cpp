// Copyright 2025 Robin Müller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @example simple_skill_build_handler.cpp
 *
 * Example for implementing a behavior tree build handler incorporating the `SimpleSkillClient` defined in
 * simple_skill_node.cpp.
 *
 * @sa <a
 * href="https://autoapms.github.io/auto-apms-guide/tutorial/creating-a-behavior-from-scratch">
 * Tutorial: Creating a Behavior From Scratch</a>
 */

/**
 * Implement the behavior tree build handler for programmatically building the simple skill tree.
 */

// This also includes the node models for all standard nodes
#include "auto_apms_behavior_tree/build_handler.hpp"

// Include custom node models (under namespace auto_apms_examples::model)
#include "auto_apms_examples/example_nodes.hpp"

namespace auto_apms_examples
{

class SimpleSkillBuildHandler : public auto_apms_behavior_tree::TreeBuildHandler
{
public:
  using TreeBuildHandler::TreeBuildHandler;

  TreeDocument::TreeElement buildTree(TreeDocument & doc, TreeBlackboard & /*bb*/) override final
  {
    // Create an empty tree element
    TreeDocument::TreeElement tree = doc.newTree("SimpleSkillDemo").makeRoot();

    // Alias for the standard node model namespace
    namespace standard_model = auto_apms_behavior_tree::model;

    // Insert the nodes of the tree using the generated node models
    TreeDocument::NodeElement sequence = tree.insertNode<standard_model::Sequence>();
    sequence.insertNode<standard_model::ForceSuccess>()
      .insertNode<model::HasParameter>()
      .set_parameter("bb.msg")
      .setConditionalScript(BT::PostCond::ON_SUCCESS, "msg := @msg")
      .setConditionalScript(BT::PostCond::ON_FAILURE, "msg := 'No blackboard parameter'");
    sequence.insertNode<standard_model::ForceSuccess>()
      .insertNode<model::HasParameter>()
      .set_parameter("bb.n_times")
      .setConditionalScript(BT::PostCond::ON_SUCCESS, "n_times := @n_times")
      .setConditionalScript(BT::PostCond::ON_FAILURE, "n_times := 1");
    sequence.insertNode<model::SimpleSkillActionNode>().set_n_times("{n_times}").set_msg("{msg}");
    sequence.insertNode<model::SimpleSkillActionNode>().set_n_times(1).set_msg("Last message");

    return tree;
  }
};

}  // namespace auto_apms_examples

// Make the build handler discoverable for the class loader
AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER(auto_apms_examples::SimpleSkillBuildHandler)