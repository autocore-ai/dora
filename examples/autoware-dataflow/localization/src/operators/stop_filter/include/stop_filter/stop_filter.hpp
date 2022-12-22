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

// Copyright 2021 TierIV
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

#ifndef STOP_FILTER__STOP_FILTER_HPP_
#define STOP_FILTER__STOP_FILTER_HPP_

#include "cxx.h"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_debug_msgs/msg/bool_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

struct StopFilterConfig;

class StopFilter
{
public:
  StopFilter(const StopFilterConfig &cfg);
  ~StopFilter(){}
  void callbackOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  nav_msgs::msg::Odometry::ConstSharedPtr get_output_odom_ptr(){
    return std::make_shared<const nav_msgs::msg::Odometry>(output_odom_msg_);
  }

private:
  nav_msgs::msg::Odometry output_odom_msg_;
  tier4_debug_msgs::msg::BoolStamped stop_flag_msg_;

  double vx_threshold_;  //!< @brief vx threshold
  double wz_threshold_;  //!< @brief wz threshold
};


std::unique_ptr<StopFilter> new_operator(const StopFilterConfig &cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(StopFilter &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);

#endif  // STOP_FILTER__STOP_FILTER_HPP_
