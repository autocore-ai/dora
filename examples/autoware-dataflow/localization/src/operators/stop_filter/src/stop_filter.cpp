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

#include "stop_filter/stop_filter.hpp"
#include "stop_filter/ffi.rs.h"
#include <iostream>

#include <algorithm>
#include <functional>
#include <utility>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_nav_msgs.hpp"

StopFilter::StopFilter(const StopFilterConfig &cfg)
{
  vx_threshold_ = cfg.vx_threshold;
  wz_threshold_ = cfg.wz_threshold;
}

void StopFilter::callbackOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  stop_flag_msg_.stamp = msg->header.stamp;
  stop_flag_msg_.data = false;

  output_odom_msg_ = *msg;
  if (
    std::fabs(msg->twist.twist.linear.x) < vx_threshold_ &&
    std::fabs(msg->twist.twist.angular.z) < wz_threshold_) {
    output_odom_msg_.twist.twist.linear.x = 0.0;
    output_odom_msg_.twist.twist.angular.z = 0.0;
    stop_flag_msg_.data = true;
  }
}

std::unique_ptr<StopFilter> new_operator(const StopFilterConfig &cfg)
{
  return std::make_unique<StopFilter>(cfg);
}

OnInputResult on_input(StopFilter &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
  // input process.  
  // std::cout << "StopFilter operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;
  
  // deseralize
  std::stringstream ss; // any(in/out) stream can be used
  copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));
  
  nav_msgs::msg::Odometry::ConstSharedPtr odom_ptr;
  {
    cereal::PortableBinaryInputArchive iarchive(ss);
    iarchive(odom_ptr);
  }
  op.callbackOdometry(odom_ptr);

  // output construct
  ss.clear();
  {
    cereal::PortableBinaryOutputArchive oarchive(ss);
    oarchive(op.get_output_odom_ptr());
  }
  std::string str = ss.str();
  // string --> unsigned char --> rust::Slice 
  auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
  rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
  auto send_result = send_output(output_sender, rust::Str("kinematic_state"), out_slice);
  OnInputResult result = {send_result.error, false};
  return result;
}
