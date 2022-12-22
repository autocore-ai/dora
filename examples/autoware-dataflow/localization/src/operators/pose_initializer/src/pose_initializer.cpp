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

// Copyright 2022 The Autoware Contributors
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

#include "pose_initializer/pose_initializer.hpp"
#include "pose_initializer/ffi.rs.h"

#include "pose_initializer/copy_vector_to_array.hpp"

#include <time_utils/time_utils.hpp>

#include <memory>
#include <vector>
#include <iostream>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_geometry_msgs.hpp"

PoseInitializer::PoseInitializer(const PoseInitializerConfig &cfg)
{
  output_pose_covariance_ = cfg.output_pose_covariance;
  gnss_particle_covariance_ = cfg.gnss_particle_covariance;
}

void PoseInitializer::change_state(InitializationState state)
{
  initialization_state_ = state;
}

void PoseInitializer::on_manual_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr manual_pose_ptr)
{
  change_state(InitializationState::INITIALIZING);
  align_request_pose_ = *manual_pose_ptr;
}

void PoseInitializer::on_fitted_gnss_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr fitted_gnss_pose_ptr)
{
  // use gnss for auto-initialization only in the UNINITIALIZED state
  if (initialization_state_ == InitializationState::UNINITIALIZED) {
    change_state(InitializationState::INITIALIZING);
    align_request_pose_ = *fitted_gnss_pose_ptr;
    align_request_pose_.pose.covariance = gnss_particle_covariance_;
    use_gnss_req_ = true;
  } else {
    use_gnss_req_ = false;
  }
}

void PoseInitializer::on_aligned_response_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr aligned_response_pose_ptr)
{
  reset_msg_ = *aligned_response_pose_ptr;
  reset_msg_.pose.covariance = output_pose_covariance_;
  change_state(InitializationState::INITIALIZED);
}


std::unique_ptr<PoseInitializer> new_operator(const PoseInitializerConfig &cfg)
{
  return std::make_unique<PoseInitializer>(cfg);
}

OnInputResult on_input(PoseInitializer &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
  // input process.  
  std::cout << "PoseInitializer operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;
  
  // deseralize
  std::stringstream ss; // any(in/out) stream can be used
  copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));
  
  if (id == "manual_pose"){
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr manual_pose_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(manual_pose_ptr);
    }
    op.on_manual_pose(manual_pose_ptr);
    // output construct
    ss.clear();
    {
      cereal::PortableBinaryOutputArchive oarchive(ss);
      oarchive(op.get_align_request_msg_ptr());
    }
    std::string str = ss.str();
    // string --> unsigned char --> rust::Slice 
    auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
    rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
    auto send_result = send_output(output_sender, rust::Str("align_request_pose"), out_slice);
    OnInputResult result = {send_result.error, false};
    return result;
  } else if (id == "fitted_gnss_pose") {
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr fitted_gnss_pose_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(fitted_gnss_pose_ptr);
    }
    op.on_fitted_gnss_pose(fitted_gnss_pose_ptr);
    if (op.use_gnss_req()){
      ss.clear();
      {
        cereal::PortableBinaryOutputArchive oarchive(ss);
        oarchive(op.get_align_request_msg_ptr());
      }
      std::string str = ss.str();
      auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
      rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
      auto send_result = send_output(output_sender, rust::Str("align_request_pose"), out_slice);
      OnInputResult result = {send_result.error, false};
      return result;
    }
  }

  if (id == "aligned_response_pose") {
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr aligned_response_pose_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(aligned_response_pose_ptr);
    }
    op.on_aligned_response_pose(aligned_response_pose_ptr);
    // output construct
    ss.clear();
    {
      cereal::PortableBinaryOutputArchive oarchive(ss);
      oarchive(op.get_reset_msg_ptr());
    }
    std::string str = ss.str();
    auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
    rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
    auto send_result = send_output(output_sender, rust::Str("initialpose3d"), out_slice);
    OnInputResult result = {send_result.error, false};
    return result;
  }
  // Set the default return
  return OnInputResult{rust::String(""), false};
}
