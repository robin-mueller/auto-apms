// Copyright 2026 Robin Müller
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

#include <gtest/gtest.h>

#include "auto_apms_behavior_tree_core/exceptions.hpp"
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

// =============================================================================
// Test Fixture
// =============================================================================

class NodeRegistrationTest : public ::testing::Test
{
protected:
  void SetUp() override { doc_ = std::make_unique<TreeDocument>(); }

  std::unique_ptr<TreeDocument> doc_;

  // The builtin Logger node from auto_apms_behavior_tree package
  static constexpr const char * LOGGER_NODE_NAME = "Logger";
  static constexpr const char * LOGGER_CLASS_NAME = "auto_apms_behavior_tree::Logger";

  // Another builtin node for testing different registrations
  static constexpr const char * ERROR_NODE_NAME = "Error";
  static constexpr const char * ERROR_CLASS_NAME = "auto_apms_behavior_tree::ThrowException";

  // Helper to create a node manifest with a single node
  static NodeManifest makeSingleNodeManifest(const std::string & node_name, const std::string & class_name)
  {
    NodeManifest::RegistrationOptions opts;
    opts.class_name = class_name;
    NodeManifest manifest;
    manifest.add(node_name, opts);
    return manifest;
  }
};

// =============================================================================
// registerNodes Basic Functionality Tests
// =============================================================================

TEST_F(NodeRegistrationTest, RegisterSingleNode)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  ASSERT_NO_THROW(doc_->registerNodes(manifest));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, RegisterMultipleNodes)
{
  NodeManifest manifest;
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = LOGGER_CLASS_NAME;
  manifest.add(LOGGER_NODE_NAME, opts1);

  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = ERROR_CLASS_NAME;
  manifest.add(ERROR_NODE_NAME, opts2);

  ASSERT_NO_THROW(doc_->registerNodes(manifest));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
  EXPECT_EQ(registered.count(ERROR_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, RegisterNodesReturnsSelf)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  TreeDocument & result = doc_->registerNodes(manifest);

  EXPECT_EQ(&result, doc_.get());
}

// =============================================================================
// Duplicate Registration Without Override Tests
// =============================================================================

TEST_F(NodeRegistrationTest, DuplicateRegistrationWithoutOverrideThrows)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  // First registration should succeed
  ASSERT_NO_THROW(doc_->registerNodes(manifest));

  // Second registration of the same node should throw without override
  EXPECT_THROW(doc_->registerNodes(manifest, false), exceptions::TreeDocumentError);
}

TEST_F(NodeRegistrationTest, DuplicateRegistrationWithDifferentClassWithoutOverrideThrows)
{
  auto manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  auto manifest2 = makeSingleNodeManifest(LOGGER_NODE_NAME, ERROR_CLASS_NAME);

  ASSERT_NO_THROW(doc_->registerNodes(manifest1));

  // Attempting to register a different class under the same name should throw
  EXPECT_THROW(doc_->registerNodes(manifest2, false), exceptions::TreeDocumentError);
}

TEST_F(NodeRegistrationTest, DuplicateRegistrationInSameManifestThrows)
{
  // NodeManifest itself prevents duplicates, so we test via separate calls
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  doc_->registerNodes(manifest);

  // Create another manifest with the same node name
  auto manifest2 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  EXPECT_THROW(doc_->registerNodes(manifest2, false), exceptions::TreeDocumentError);
}

TEST_F(NodeRegistrationTest, PartialDuplicateInManifestThrows)
{
  // Register one node
  auto manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest1);

  // Try to register a manifest containing a new node AND a duplicate
  NodeManifest manifest2;
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = ERROR_CLASS_NAME;
  manifest2.add(ERROR_NODE_NAME, opts1);

  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = LOGGER_CLASS_NAME;
  manifest2.add(LOGGER_NODE_NAME, opts2);  // This is a duplicate

  // Should throw because of the duplicate, even though ERROR_NODE_NAME is new
  EXPECT_THROW(doc_->registerNodes(manifest2, false), exceptions::TreeDocumentError);
}

// =============================================================================
// Override Registration Tests
// =============================================================================

TEST_F(NodeRegistrationTest, DuplicateRegistrationWithOverrideSucceeds)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  ASSERT_NO_THROW(doc_->registerNodes(manifest));

  // Second registration with override=true should succeed
  ASSERT_NO_THROW(doc_->registerNodes(manifest, true));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, OverrideWithDifferentClassSucceeds)
{
  auto manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest1);

  // Create a manifest using the same registration name but different class
  auto manifest2 = makeSingleNodeManifest(LOGGER_NODE_NAME, ERROR_CLASS_NAME);

  // With override=true, this should succeed
  ASSERT_NO_THROW(doc_->registerNodes(manifest2, true));

  // Verify the node is still registered
  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, OverrideUpdatesFactory)
{
  // Register Logger node under the "MyNode" name
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = LOGGER_CLASS_NAME;
  NodeManifest manifest1;
  manifest1.add("MyNode", opts1);
  doc_->registerNodes(manifest1);

  // Now override with Error node under the same "MyNode" name
  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = ERROR_CLASS_NAME;
  NodeManifest manifest2;
  manifest2.add("MyNode", opts2);

  ASSERT_NO_THROW(doc_->registerNodes(manifest2, true));

  // Create a tree using the overridden node
  doc_->mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <MyNode/>
    </BehaviorTree>
  </root>)");

  // Verify the tree can be verified (node is usable)
  auto result = doc_->verify();
  EXPECT_TRUE(result);
}

TEST_F(NodeRegistrationTest, MultipleOverrides)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  // Register the same node multiple times with override=true
  doc_->registerNodes(manifest);
  ASSERT_NO_THROW(doc_->registerNodes(manifest, true));
  ASSERT_NO_THROW(doc_->registerNodes(manifest, true));
  ASSERT_NO_THROW(doc_->registerNodes(manifest, true));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, PartialOverrideInManifest)
{
  // Register one node
  auto manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest1);

  // Register a manifest containing a new node AND a duplicate, with override=true
  NodeManifest manifest2;
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = ERROR_CLASS_NAME;
  manifest2.add(ERROR_NODE_NAME, opts1);

  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = LOGGER_CLASS_NAME;
  manifest2.add(LOGGER_NODE_NAME, opts2);  // This would be a duplicate

  // With override=true, this should succeed
  ASSERT_NO_THROW(doc_->registerNodes(manifest2, true));

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
  EXPECT_EQ(registered.count(ERROR_NODE_NAME), 1u);
}

// =============================================================================
// Native Node Name Reservation Tests
// =============================================================================

TEST_F(NodeRegistrationTest, RegisterNativeNodeNameThrows)
{
  // Attempting to register a node with a name reserved for native nodes should throw
  NodeManifest::RegistrationOptions opts;
  opts.class_name = LOGGER_CLASS_NAME;
  NodeManifest manifest;
  manifest.add("Sequence", opts);  // "Sequence" is a native node name

  EXPECT_THROW(doc_->registerNodes(manifest), exceptions::TreeDocumentError);
}

TEST_F(NodeRegistrationTest, RegisterMultipleNodesWithOneNativeNameThrows)
{
  NodeManifest manifest;

  // Valid node
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = LOGGER_CLASS_NAME;
  manifest.add(LOGGER_NODE_NAME, opts1);

  // Reserved native name
  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = ERROR_CLASS_NAME;
  manifest.add("Fallback", opts2);  // "Fallback" is a native node name

  EXPECT_THROW(doc_->registerNodes(manifest), exceptions::TreeDocumentError);
}

// =============================================================================
// Integration Tests with Tree Usage
// =============================================================================

TEST_F(NodeRegistrationTest, RegisteredNodeCanBeUsedInTree)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  doc_->registerNodes(manifest);

  doc_->mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <Logger message="Hello"/>
    </BehaviorTree>
  </root>)");

  auto result = doc_->verify();
  EXPECT_TRUE(result);
}

TEST_F(NodeRegistrationTest, UnregisteredNodeInTreeFailsVerification)
{
  // Don't register any nodes
  doc_->mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <Logger message="Hello"/>
    </BehaviorTree>
  </root>)");

  auto result = doc_->verify();
  EXPECT_FALSE(result);
}

TEST_F(NodeRegistrationTest, OverriddenNodeWorksInTree)
{
  // First register Logger under custom name
  NodeManifest::RegistrationOptions opts1;
  opts1.class_name = LOGGER_CLASS_NAME;
  NodeManifest manifest1;
  manifest1.add("CustomAction", opts1);
  doc_->registerNodes(manifest1);

  // Override with Error node
  NodeManifest::RegistrationOptions opts2;
  opts2.class_name = ERROR_CLASS_NAME;
  NodeManifest manifest2;
  manifest2.add("CustomAction", opts2);
  doc_->registerNodes(manifest2, true);

  // Create tree using the overridden node
  doc_->mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <CustomAction message="Test"/>
    </BehaviorTree>
  </root>)");

  auto result = doc_->verify();
  EXPECT_TRUE(result);
}

// =============================================================================
// Method Chaining Tests
// =============================================================================

TEST_F(NodeRegistrationTest, RegisterNodesMethodChaining)
{
  NodeManifest manifest1 = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);
  NodeManifest manifest2 = makeSingleNodeManifest(ERROR_NODE_NAME, ERROR_CLASS_NAME);

  doc_->registerNodes(manifest1).registerNodes(manifest2);

  auto registered = doc_->getRegisteredNodeNames(false);
  EXPECT_EQ(registered.count(LOGGER_NODE_NAME), 1u);
  EXPECT_EQ(registered.count(ERROR_NODE_NAME), 1u);
}

TEST_F(NodeRegistrationTest, RegisterNodesAndBuildTreeChaining)
{
  auto manifest = makeSingleNodeManifest(LOGGER_NODE_NAME, LOGGER_CLASS_NAME);

  doc_->registerNodes(manifest).mergeString(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
    <BehaviorTree ID="TestTree">
      <Logger message="Hello"/>
    </BehaviorTree>
  </root>)");

  EXPECT_TRUE(doc_->hasTreeName("TestTree"));
  auto result = doc_->verify();
  EXPECT_TRUE(result);
}
