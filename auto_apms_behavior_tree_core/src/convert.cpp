// Copyright 2024 Robin Müller
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

#include "auto_apms_behavior_tree_core/convert.hpp"

#include <sstream>

#include "auto_apms_util/string.hpp"

/// @cond INTERNAL
namespace BT
{

template <>
std::vector<uint8_t> convertFromString<std::vector<uint8_t>>(StringView str)
{
  auto parts = BT::splitString(str, ';');
  std::vector<uint8_t> output;
  output.reserve(parts.size());
  for (const StringView & part : parts) {
    output.push_back(convertFromString<uint8_t>(part));
  }
  return output;
}

template <>
std::vector<bool> convertFromString<std::vector<bool>>(StringView str)
{
  auto parts = BT::splitString(str, ';');
  std::vector<bool> output;
  output.reserve(parts.size());
  for (const StringView & part : parts) {
    output.push_back(convertFromString<bool>(part));
  }
  return output;
}

template <>
std::vector<int> convertFromString<std::vector<int>>(StringView str)
{
  auto parts = BT::splitString(str, ';');
  std::vector<int, std::allocator<int>> output;
  output.reserve(parts.size());
  for (const StringView & part : parts) {
    output.push_back(convertFromString<int>(part));
  }
  return output;
}

template <>
std::vector<float> convertFromString<std::vector<float>>(StringView str)
{
  auto parts = splitString(str, ';');
  std::vector<float> output;
  output.reserve(parts.size());
  for (const StringView & part : parts) {
    output.push_back(convertFromString<float>(part));
  }
  return output;
}

template <>
Eigen::MatrixXd convertFromString<Eigen::MatrixXd>(StringView str)
{
  const std::string input_str(str);
  std::vector<std::vector<double>> values;
  std::stringstream matrix_stream(input_str);
  std::string row_str;

  // Parse each row (separated by commas)
  while (std::getline(matrix_stream, row_str, ',')) {
    std::stringstream row_stream(row_str);
    std::string value_str;
    std::vector<double> row_values;

    // Parse each value in the row (separated by semicolons)
    while (std::getline(row_stream, value_str, ';')) {
      row_values.push_back(convertFromString<double>(value_str));
    }

    if (!row_values.empty()) {
      values.push_back(row_values);
    }
  }

  // Create the matrix
  if (values.empty()) {
    return Eigen::MatrixXd();
  }
  size_t rows = values.size();
  size_t cols = values[0].size();
  Eigen::MatrixXd matrix(rows, cols);

  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < std::min(cols, values[i].size()); ++j) {
      matrix(i, j) = values[i][j];
    }
  }
  return matrix;
}

template <>
std::string toStr(const std::vector<uint8_t> & value)
{
  return auto_apms_util::join(value, ";");
}

template <>
std::string toStr<std::vector<bool>>(const std::vector<bool> & value)
{
  return auto_apms_util::join(value, ";");
}

template <>
std::string toStr<std::vector<int>>(const std::vector<int> & value)
{
  return auto_apms_util::join(value, ";");
}

template <>
std::string toStr<std::vector<double>>(const std::vector<double> & value)
{
  return auto_apms_util::join(value, ";");
}

template <>
std::string toStr<Eigen::MatrixXd>(const Eigen::MatrixXd & value)
{
  Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ";", ",", "", "", "", "");
  std::stringstream ss;
  ss << value.format(fmt);
  return ss.str();
}

template <>
std::string toStr<std::vector<std::string>>(const std::vector<std::string> & value)
{
  return auto_apms_util::join(value, ";");
}

}  // namespace BT
/// @endcond
