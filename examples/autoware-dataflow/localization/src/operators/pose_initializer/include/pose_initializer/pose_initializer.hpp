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

#ifndef POSE_INITIALIZER__POSE_INITIALIZER_HPP_
#define POSE_INITIALIZER__POSE_INITIALIZER_HPP_

#include "cxx.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <memory>

struct PoseInitializerConfig;

enum class InitializationState {
  UNKNOWN = 0,
  UNINITIALIZED = 1,
  INITIALIZING = 2,
  INITIALIZED = 3,
};

class PoseInitializer
{
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  PoseInitializer(const PoseInitializerConfig &cfg);
  ~PoseInitializer(){}

  PoseWithCovarianceStamped::ConstSharedPtr get_align_request_msg_ptr(){
    return std::make_shared<const PoseWithCovarianceStamped>(align_request_pose_);
  }
  PoseWithCovarianceStamped::ConstSharedPtr get_reset_msg_ptr(){
    // return std::make_shared<const PoseWithCovarianceStamped>(reset_msg_);
    return std::make_shared<PoseWithCovarianceStamped>(reset_msg_);
  }
  bool use_gnss_req(){return use_gnss_req_;}

  void on_manual_pose(PoseWithCovarianceStamped::ConstSharedPtr manual_pose_ptr);
  void on_fitted_gnss_pose(PoseWithCovarianceStamped::ConstSharedPtr fitted_gnss_pose_ptr);
  void on_aligned_response_pose(PoseWithCovarianceStamped::ConstSharedPtr aligned_response_pose_ptr);

private:
  InitializationState initialization_state_ = InitializationState::UNINITIALIZED;
  bool use_gnss_req_ = false;

  PoseWithCovarianceStamped align_request_pose_;
  PoseWithCovarianceStamped reset_msg_;
  std::array<double, 36> output_pose_covariance_;
  std::array<double, 36> gnss_particle_covariance_;

  void change_state(InitializationState state);
};

std::unique_ptr<PoseInitializer> new_operator(const PoseInitializerConfig &cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(PoseInitializer &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);

#endif  // POSE_INITIALIZER__POSE_INITIALIZER_HPP_
