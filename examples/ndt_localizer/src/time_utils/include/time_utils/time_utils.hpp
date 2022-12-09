/*
 *  Copyright(c) 2021 to 2023 AutoCore Technology (Nanjing) Co., Ltd. All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 *    conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 *    of conditions and the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific prior written
 *    permission.
 */

// Copyright 2019-2021 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#ifndef TIME_UTILS__TIME_UTILS_HPP_
#define TIME_UTILS__TIME_UTILS_HPP_

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <time_utils/visibility_control.hpp>

#include <chrono>

namespace time_utils
{
/// Convert from std::chrono::time_point to time message
TIME_UTILS_PUBLIC builtin_interfaces::msg::Time to_message(std::chrono::system_clock::time_point t);
/// Convert from std::chrono::duration to duration message
TIME_UTILS_PUBLIC builtin_interfaces::msg::Duration to_message(std::chrono::nanoseconds dt);
/// Convert from std::chrono::time_point from time message
TIME_UTILS_PUBLIC
std::chrono::system_clock::time_point from_message(builtin_interfaces::msg::Time t) noexcept;
/// Convert from std::chrono::duration from duration message
TIME_UTILS_PUBLIC
std::chrono::nanoseconds from_message(builtin_interfaces::msg::Duration dt) noexcept;
/// Standard interpolation
TIME_UTILS_PUBLIC std::chrono::nanoseconds interpolate(
  std::chrono::nanoseconds a, std::chrono::nanoseconds b, float t) noexcept;

namespace details
{
template <typename TimeT>
TimeT duration_to_msg(std::chrono::nanoseconds dt);
}  // namespace details
}  // namespace time_utils

#endif  // TIME_UTILS__TIME_UTILS_HPP_
