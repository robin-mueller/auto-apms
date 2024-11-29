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

#include "auto_apms_behavior_tree/util/conversion.hpp"

#include "auto_apms_behavior_tree/exceptions.hpp"
#include "auto_apms_util/string.hpp"

/// @cond
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
std::vector<int64_t> convertFromString<std::vector<int64_t>>(StringView str)
{
  auto parts = BT::splitString(str, ';');
  std::vector<int64_t, std::allocator<int64_t>> output;
  output.reserve(parts.size());
  for (const StringView & part : parts) {
    output.push_back(convertFromString<int64_t>(part));
  }
  return output;
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
std::string toStr<std::vector<int64_t>>(const std::vector<int64_t> & value)
{
  return auto_apms_util::join(value, ";");
}

template <>
std::string toStr<std::vector<double>>(const std::vector<double> & value)
{
  return auto_apms_util::join(value, ";");
}

template <>
std::string toStr<std::vector<std::string>>(const std::vector<std::string> & value)
{
  return auto_apms_util::join(value, ";");
}

}  // namespace BT
/// @endcond

namespace auto_apms_behavior_tree
{

BT::Expected<BT::Any> createAnyFromParameterValue(const rclcpp::ParameterValue & val)
{
  switch (val.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return BT::GetAnyFromStringFunctor<bool>()(BT::toStr(val.get<bool>()));
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return BT::GetAnyFromStringFunctor<int64_t>()(BT::toStr(val.get<int64_t>()));
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return BT::GetAnyFromStringFunctor<double>()(BT::toStr(val.get<double>()));
    case rclcpp::ParameterType::PARAMETER_STRING:
      return BT::GetAnyFromStringFunctor<std::string>()(BT::toStr(val.get<std::string>()));
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return BT::GetAnyFromStringFunctor<std::vector<uint8_t>>()(BT::toStr(val.get<std::vector<uint8_t>>()));
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return BT::GetAnyFromStringFunctor<std::vector<bool>>()(BT::toStr(val.get<std::vector<bool>>()));
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return BT::GetAnyFromStringFunctor<std::vector<int64_t>>()(BT::toStr(val.get<std::vector<int64_t>>()));
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return BT::GetAnyFromStringFunctor<std::vector<double>>()(BT::toStr(val.get<std::vector<double>>()));
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return BT::GetAnyFromStringFunctor<std::vector<std::string>>()(BT::toStr(val.get<std::vector<std::string>>()));
    default:
      break;
  }

  // Cannot convert
  return nonstd::make_unexpected(
    "Conversion of rclcpp::ParameterValue with type '" + rclcpp::to_string(val.get_type()) +
    "' to BT::Any is undefined.");
}

BT::Expected<rclcpp::ParameterValue> createParameterValueFromAny(const BT::Any & any, rclcpp::ParameterType type)
{
  std::string error;
  switch (type) {
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      // If type is not set, try different casts and use the first one
      if (any.isType<bool>()) return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_BOOL);
      if (any.isType<int64_t>()) return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_INTEGER);
      if (any.isType<double>()) return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_DOUBLE);
      if (any.isType<std::string>()) return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_STRING);
      if (any.isType<std::vector<uint8_t>>())
        return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_BYTE_ARRAY);
      if (any.isType<std::vector<bool>>())
        return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_BOOL_ARRAY);
      if (any.isType<std::vector<int64_t>>())
        return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY);
      if (any.isType<std::vector<double>>())
        return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
      if (any.isType<std::vector<std::string>>())
        return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_STRING_ARRAY);

      // If none of the usual types, try internal conversions
      if (any.isIntegral()) return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_INTEGER);
      if (any.isNumber()) return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_DOUBLE);
      if (any.isString()) return createParameterValueFromAny(any, rclcpp::ParameterType::PARAMETER_STRING);

      error = "Cannot infer parameter type from BT::Any.";
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL: {
      const auto casted = any.tryCast<bool>();
      if (casted) return rclcpp::ParameterValue(casted.value());
      error = casted.error();
      break;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER: {
      const auto casted = any.tryCast<int64_t>();
      if (casted) return rclcpp::ParameterValue(casted.value());
      error = casted.error();
      break;
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE: {
      const auto casted = any.tryCast<double>();
      if (casted) return rclcpp::ParameterValue(casted.value());
      error = casted.error();
      break;
    }
    case rclcpp::ParameterType::PARAMETER_STRING: {
      const auto casted = any.tryCast<std::string>();
      if (casted) return rclcpp::ParameterValue(casted.value());
      error = casted.error();
      break;
    }
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY: {
      const auto casted = any.tryCast<std::vector<uint8_t>>();
      if (casted) return rclcpp::ParameterValue(casted.value());
      error = casted.error();
      break;
    }
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
      const auto casted = any.tryCast<std::vector<bool>>();
      if (casted) return rclcpp::ParameterValue(casted.value());
      error = casted.error();
      break;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      const auto casted = any.tryCast<std::vector<int64_t>>();
      if (casted) return rclcpp::ParameterValue(casted.value());
      error = casted.error();
      break;
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      const auto casted = any.tryCast<std::vector<double>>();
      if (casted) return rclcpp::ParameterValue(casted.value());
      error = casted.error();
      break;
    }
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
      const auto casted = any.tryCast<std::vector<std::string>>();
      if (casted) return rclcpp::ParameterValue(casted.value());
      error = casted.error();
      break;
    }
  }

  // Cannot convert
  return nonstd::make_unexpected(
    "Conversion of BT::Any (Internal type: " + BT::demangle(any.type()) + ") to rclcpp::ParameterValue with type '" +
    rclcpp::to_string(type) + "' failed: " + error);
}

}  // namespace auto_apms_behavior_tree