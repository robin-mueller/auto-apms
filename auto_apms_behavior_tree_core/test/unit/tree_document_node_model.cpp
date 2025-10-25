// Copyright 2025 Robin Müller
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

#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"
#include "behaviortree_cpp/basic_types.h"

using namespace auto_apms_behavior_tree::core;
using namespace auto_apms_behavior_tree;

class TreeDocumentNodeModelTest : public ::testing::Test
{
protected:
  void SetUp() override { doc_ = std::make_unique<TreeDocument>(); }

  std::unique_ptr<TreeDocument> doc_;
};

TEST_F(TreeDocumentNodeModelTest, AddEmptyNodeModel)
{
  // Test adding an empty node model map
  NodeModelMap empty_map;
  ASSERT_NO_THROW(doc_->addNodeModel(empty_map));

  // Verify the document still writes correctly
  std::string xml = doc_->writeToString();
  // When an empty model is added, a TreeNodesModel element should still be created
  EXPECT_TRUE(xml.find("<TreeNodesModel") != std::string::npos);
}

TEST_F(TreeDocumentNodeModelTest, AddSingleActionNodeWithoutPorts)
{
  // Create a simple action node model without ports
  NodeModelMap model_map;
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;
  model_map["SimpleAction"] = action_model;

  ASSERT_NO_THROW(doc_->addNodeModel(model_map));

  // Verify the XML output contains the node
  std::string xml = doc_->writeToString();
  EXPECT_TRUE(xml.find("<TreeNodesModel>") != std::string::npos);
  EXPECT_TRUE(xml.find("<Action ID=\"SimpleAction\"") != std::string::npos);
}

TEST_F(TreeDocumentNodeModelTest, AddActionNodeWithInputPort)
{
  // Create an action node with an input port
  NodeModelMap model_map;
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;

  NodePortInfo input_port;
  input_port.port_name = "target";
  input_port.port_type = "std::string";
  input_port.port_direction = BT::PortDirection::INPUT;
  input_port.port_has_default = false;
  input_port.port_description = "Target position";

  action_model.port_infos.push_back(input_port);
  model_map["MoveToTarget"] = action_model;

  ASSERT_NO_THROW(doc_->addNodeModel(model_map));

  // Verify the XML output
  std::string xml = doc_->writeToString();
  EXPECT_TRUE(xml.find("<Action ID=\"MoveToTarget\"") != std::string::npos);
  EXPECT_TRUE(xml.find("<input_port name=\"target\"") != std::string::npos);
  EXPECT_TRUE(xml.find("type=\"std::string\"") != std::string::npos);
  EXPECT_TRUE(xml.find("Target position") != std::string::npos);
}

TEST_F(TreeDocumentNodeModelTest, AddConditionNodeWithOutputPort)
{
  // Create a condition node with an output port
  NodeModelMap model_map;
  NodeModel condition_model;
  condition_model.type = BT::NodeType::CONDITION;

  NodePortInfo output_port;
  output_port.port_name = "result";
  output_port.port_type = "bool";
  output_port.port_direction = BT::PortDirection::OUTPUT;
  output_port.port_has_default = false;
  output_port.port_description = "Condition result";

  condition_model.port_infos.push_back(output_port);
  model_map["CheckCondition"] = condition_model;

  ASSERT_NO_THROW(doc_->addNodeModel(model_map));

  // Verify the XML output
  std::string xml = doc_->writeToString();
  EXPECT_TRUE(xml.find("<Condition ID=\"CheckCondition\"") != std::string::npos);
  EXPECT_TRUE(xml.find("<output_port name=\"result\"") != std::string::npos);
  EXPECT_TRUE(xml.find("type=\"bool\"") != std::string::npos);
}

TEST_F(TreeDocumentNodeModelTest, AddNodeWithInoutPort)
{
  // Create a node with an inout port
  NodeModelMap model_map;
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;

  NodePortInfo inout_port;
  inout_port.port_name = "data";
  inout_port.port_type = "int";
  inout_port.port_direction = BT::PortDirection::INOUT;
  inout_port.port_has_default = false;
  inout_port.port_description = "Data to read and write";

  action_model.port_infos.push_back(inout_port);
  model_map["ProcessData"] = action_model;

  ASSERT_NO_THROW(doc_->addNodeModel(model_map));

  // Verify the XML output
  std::string xml = doc_->writeToString();
  EXPECT_TRUE(xml.find("<inout_port name=\"data\"") != std::string::npos);
  EXPECT_TRUE(xml.find("type=\"int\"") != std::string::npos);
}

TEST_F(TreeDocumentNodeModelTest, AddNodeWithDefaultValue)
{
  // Create a node with a port that has a default value
  NodeModelMap model_map;
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;

  NodePortInfo input_port;
  input_port.port_name = "timeout";
  input_port.port_type = "double";
  input_port.port_direction = BT::PortDirection::INPUT;
  input_port.port_has_default = true;
  input_port.port_default = "5.0";
  input_port.port_description = "Timeout in seconds";

  action_model.port_infos.push_back(input_port);
  model_map["WaitAction"] = action_model;

  ASSERT_NO_THROW(doc_->addNodeModel(model_map));

  // Verify the XML output
  std::string xml = doc_->writeToString();
  EXPECT_TRUE(xml.find("default=\"5.0\"") != std::string::npos);
}

TEST_F(TreeDocumentNodeModelTest, AddMultipleNodesWithDifferentTypes)
{
  // Create multiple nodes with different types
  NodeModelMap model_map;

  // Action node
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;
  model_map["MyAction"] = action_model;

  // Condition node
  NodeModel condition_model;
  condition_model.type = BT::NodeType::CONDITION;
  model_map["MyCondition"] = condition_model;

  // Decorator node
  NodeModel decorator_model;
  decorator_model.type = BT::NodeType::DECORATOR;
  model_map["MyDecorator"] = decorator_model;

  // Control node
  NodeModel control_model;
  control_model.type = BT::NodeType::CONTROL;
  model_map["MySequence"] = control_model;

  ASSERT_NO_THROW(doc_->addNodeModel(model_map));

  // Verify all nodes are in the XML
  std::string xml = doc_->writeToString();
  EXPECT_TRUE(xml.find("<Action ID=\"MyAction\"") != std::string::npos);
  EXPECT_TRUE(xml.find("<Condition ID=\"MyCondition\"") != std::string::npos);
  EXPECT_TRUE(xml.find("<Decorator ID=\"MyDecorator\"") != std::string::npos);
  EXPECT_TRUE(xml.find("<Control ID=\"MySequence\"") != std::string::npos);
}

TEST_F(TreeDocumentNodeModelTest, AddNodeWithMultiplePorts)
{
  // Create a node with multiple ports of different types
  NodeModelMap model_map;
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;

  NodePortInfo input_port1;
  input_port1.port_name = "x";
  input_port1.port_type = "double";
  input_port1.port_direction = BT::PortDirection::INPUT;
  input_port1.port_has_default = true;
  input_port1.port_default = "0.0";
  input_port1.port_description = "X coordinate";

  NodePortInfo input_port2;
  input_port2.port_name = "y";
  input_port2.port_type = "double";
  input_port2.port_direction = BT::PortDirection::INPUT;
  input_port2.port_has_default = true;
  input_port2.port_default = "0.0";
  input_port2.port_description = "Y coordinate";

  NodePortInfo output_port;
  output_port.port_name = "success";
  output_port.port_type = "bool";
  output_port.port_direction = BT::PortDirection::OUTPUT;
  output_port.port_has_default = false;
  output_port.port_description = "Whether the action succeeded";

  action_model.port_infos.push_back(input_port1);
  action_model.port_infos.push_back(input_port2);
  action_model.port_infos.push_back(output_port);
  model_map["MoveToPosition"] = action_model;

  ASSERT_NO_THROW(doc_->addNodeModel(model_map));

  // Verify all ports are in the XML
  std::string xml = doc_->writeToString();
  EXPECT_TRUE(xml.find("<input_port name=\"x\"") != std::string::npos);
  EXPECT_TRUE(xml.find("<input_port name=\"y\"") != std::string::npos);
  EXPECT_TRUE(xml.find("<output_port name=\"success\"") != std::string::npos);
}

TEST_F(TreeDocumentNodeModelTest, AddNodeModelToDocumentWithExistingModel)
{
  // First, add a node model map
  NodeModelMap model_map1;
  NodeModel action_model1;
  action_model1.type = BT::NodeType::ACTION;
  model_map1["FirstAction"] = action_model1;

  ASSERT_NO_THROW(doc_->addNodeModel(model_map1));

  // Now add another node model map
  NodeModelMap model_map2;
  NodeModel action_model2;
  action_model2.type = BT::NodeType::ACTION;
  model_map2["SecondAction"] = action_model2;

  ASSERT_NO_THROW(doc_->addNodeModel(model_map2));

  // Verify both nodes are in the XML
  std::string xml = doc_->writeToString();
  EXPECT_TRUE(xml.find("<Action ID=\"FirstAction\"") != std::string::npos);
  EXPECT_TRUE(xml.find("<Action ID=\"SecondAction\"") != std::string::npos);
}

TEST_F(TreeDocumentNodeModelTest, VerifyXMLStructure)
{
  // Create a complete node model
  NodeModelMap model_map;
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;

  NodePortInfo port;
  port.port_name = "message";
  port.port_type = "std::string";
  port.port_direction = BT::PortDirection::INPUT;
  port.port_has_default = true;
  port.port_default = "Hello";
  port.port_description = "Message to print";

  action_model.port_infos.push_back(port);
  model_map["PrintMessage"] = action_model;

  doc_->addNodeModel(model_map);

  // Verify the complete XML structure
  std::string xml = doc_->writeToString();

  // Check root element
  EXPECT_TRUE(xml.find("<root ") != std::string::npos);
  EXPECT_TRUE(xml.find("BTCPP_format=") != std::string::npos);

  // Check TreeNodesModel element exists
  EXPECT_TRUE(xml.find("<TreeNodesModel>") != std::string::npos);
  EXPECT_TRUE(xml.find("</TreeNodesModel>") != std::string::npos);

  // Check node definition
  EXPECT_TRUE(xml.find("<Action ID=\"PrintMessage\"") != std::string::npos);
  EXPECT_TRUE(xml.find("</Action>") != std::string::npos);

  // Check port definition
  EXPECT_TRUE(xml.find("<input_port") != std::string::npos);
  EXPECT_TRUE(xml.find("name=\"message\"") != std::string::npos);
  EXPECT_TRUE(xml.find("type=\"std::string\"") != std::string::npos);
  EXPECT_TRUE(xml.find("default=\"Hello\"") != std::string::npos);
  EXPECT_TRUE(xml.find("Message to print") != std::string::npos);
}

TEST_F(TreeDocumentNodeModelTest, RoundTripNodeModelConversion)
{
  // Create a node model, add it to the document, then parse it back
  NodeModelMap original_map;
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;

  NodePortInfo port;
  port.port_name = "value";
  port.port_type = "int";
  port.port_direction = BT::PortDirection::INPUT;
  port.port_has_default = true;
  port.port_default = "42";
  port.port_description = "An integer value";

  action_model.port_infos.push_back(port);
  original_map["TestNode"] = action_model;

  doc_->addNodeModel(original_map);

  // Get the XML string
  std::string xml = doc_->writeToString();

  // Create a new document from the XML and parse the node model back
  tinyxml2::XMLDocument xml_doc;
  ASSERT_EQ(xml_doc.Parse(xml.c_str()), tinyxml2::XMLError::XML_SUCCESS);

  NodeModelMap parsed_map;
  ASSERT_NO_THROW(parsed_map = TreeDocument::getNodeModel(xml_doc));

  // Verify the parsed model matches the original
  ASSERT_EQ(parsed_map.size(), 1);
  ASSERT_TRUE(parsed_map.find("TestNode") != parsed_map.end());

  const NodeModel & parsed_model = parsed_map["TestNode"];
  EXPECT_EQ(parsed_model.type, BT::NodeType::ACTION);
  ASSERT_EQ(parsed_model.port_infos.size(), 1);

  const NodePortInfo & parsed_port = parsed_model.port_infos[0];
  EXPECT_EQ(parsed_port.port_name, "value");
  EXPECT_EQ(parsed_port.port_type, "int");
  EXPECT_EQ(parsed_port.port_direction, BT::PortDirection::INPUT);
  EXPECT_TRUE(parsed_port.port_has_default);
  EXPECT_EQ(parsed_port.port_default, "42");
  EXPECT_EQ(parsed_port.port_description, "An integer value");
}

TEST_F(TreeDocumentNodeModelTest, GetNodeModelWithHiddenPorts)
{
  // Create a node model with multiple ports
  NodeModelMap model_map;
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;

  NodePortInfo port1;
  port1.port_name = "visible_port";
  port1.port_type = "std::string";
  port1.port_direction = BT::PortDirection::INPUT;
  port1.port_has_default = false;
  port1.port_description = "A visible port";

  NodePortInfo port2;
  port2.port_name = "hidden_port";
  port2.port_type = "int";
  port2.port_direction = BT::PortDirection::INPUT;
  port2.port_has_default = false;
  port2.port_description = "A port that will be hidden";

  NodePortInfo port3;
  port3.port_name = "another_visible";
  port3.port_type = "double";
  port3.port_direction = BT::PortDirection::OUTPUT;
  port3.port_has_default = false;
  port3.port_description = "Another visible port";

  action_model.port_infos.push_back(port1);
  action_model.port_infos.push_back(port2);
  action_model.port_infos.push_back(port3);

  model_map["TestNodeWithHiddenPorts"] = action_model;

  // Add the model to a document and write to XML
  doc_->addNodeModel(model_map);
  std::string xml = doc_->writeToString();

  // Parse the XML back
  tinyxml2::XMLDocument xml_doc;
  ASSERT_EQ(xml_doc.Parse(xml.c_str()), tinyxml2::XMLError::XML_SUCCESS);

  // Now test getNodeModel with hidden ports specified
  std::map<std::string, std::vector<std::string>> hidden_ports;
  hidden_ports["TestNodeWithHiddenPorts"] = {"hidden_port"};

  NodeModelMap filtered_map;
  ASSERT_NO_THROW(filtered_map = TreeDocument::getNodeModel(xml_doc, hidden_ports));

  // Verify the hidden port was removed
  ASSERT_TRUE(filtered_map.find("TestNodeWithHiddenPorts") != filtered_map.end());
  const NodeModel & filtered_model = filtered_map["TestNodeWithHiddenPorts"];
  ASSERT_EQ(filtered_model.port_infos.size(), 2);

  // Check that the remaining ports are the visible ones
  bool has_visible_port = false;
  bool has_another_visible = false;
  bool has_hidden_port = false;

  for (const auto & port : filtered_model.port_infos) {
    if (port.port_name == "visible_port") has_visible_port = true;
    if (port.port_name == "another_visible") has_another_visible = true;
    if (port.port_name == "hidden_port") has_hidden_port = true;
  }

  EXPECT_TRUE(has_visible_port);
  EXPECT_TRUE(has_another_visible);
  EXPECT_FALSE(has_hidden_port);
}

TEST_F(TreeDocumentNodeModelTest, GetNodeModelWithMultipleHiddenPorts)
{
  // Create a node with multiple ports to hide
  NodeModelMap model_map;
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;

  std::vector<std::string> port_names = {"port1", "port2", "port3", "port4", "port5"};
  for (const auto & name : port_names) {
    NodePortInfo port;
    port.port_name = name;
    port.port_type = "std::string";
    port.port_direction = BT::PortDirection::INPUT;
    port.port_has_default = false;
    port.port_description = "Port " + name;
    action_model.port_infos.push_back(port);
  }

  model_map["MultiPortNode"] = action_model;

  // Add to document and convert to XML
  doc_->addNodeModel(model_map);
  std::string xml = doc_->writeToString();

  // Parse the XML back
  tinyxml2::XMLDocument xml_doc;
  ASSERT_EQ(xml_doc.Parse(xml.c_str()), tinyxml2::XMLError::XML_SUCCESS);

  // Hide multiple ports
  std::map<std::string, std::vector<std::string>> hidden_ports;
  hidden_ports["MultiPortNode"] = {"port2", "port4"};

  NodeModelMap filtered_map;
  ASSERT_NO_THROW(filtered_map = TreeDocument::getNodeModel(xml_doc, hidden_ports));

  // Verify only 3 ports remain (port1, port3, port5)
  ASSERT_TRUE(filtered_map.find("MultiPortNode") != filtered_map.end());
  const NodeModel & filtered_model = filtered_map["MultiPortNode"];
  ASSERT_EQ(filtered_model.port_infos.size(), 3);

  // Verify the correct ports remain
  std::set<std::string> remaining_ports;
  for (const auto & port : filtered_model.port_infos) {
    remaining_ports.insert(port.port_name);
  }

  EXPECT_TRUE(remaining_ports.find("port1") != remaining_ports.end());
  EXPECT_TRUE(remaining_ports.find("port3") != remaining_ports.end());
  EXPECT_TRUE(remaining_ports.find("port5") != remaining_ports.end());
  EXPECT_FALSE(remaining_ports.find("port2") != remaining_ports.end());
  EXPECT_FALSE(remaining_ports.find("port4") != remaining_ports.end());
}

TEST_F(TreeDocumentNodeModelTest, GetNodeModelWithEmptyHiddenPorts)
{
  // Test that empty hidden_ports map doesn't affect the result
  NodeModelMap model_map;
  NodeModel action_model;
  action_model.type = BT::NodeType::ACTION;

  NodePortInfo port;
  port.port_name = "test_port";
  port.port_type = "int";
  port.port_direction = BT::PortDirection::INPUT;
  port.port_has_default = false;
  port.port_description = "Test port";

  action_model.port_infos.push_back(port);
  model_map["TestNode"] = action_model;

  // Add to document and convert to XML
  doc_->addNodeModel(model_map);
  std::string xml = doc_->writeToString();

  // Parse the XML back
  tinyxml2::XMLDocument xml_doc;
  ASSERT_EQ(xml_doc.Parse(xml.c_str()), tinyxml2::XMLError::XML_SUCCESS);

  // Call getNodeModel with empty hidden_ports map
  std::map<std::string, std::vector<std::string>> hidden_ports;

  NodeModelMap result_map;
  ASSERT_NO_THROW(result_map = TreeDocument::getNodeModel(xml_doc, hidden_ports));

  // Verify the port is still there
  ASSERT_TRUE(result_map.find("TestNode") != result_map.end());
  const NodeModel & result_model = result_map["TestNode"];
  ASSERT_EQ(result_model.port_infos.size(), 1);
  EXPECT_EQ(result_model.port_infos[0].port_name, "test_port");
}
