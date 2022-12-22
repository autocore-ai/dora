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

// Copyright 2020 Tier IV, Inc.
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
#ifndef IMU_CORRECTOR__IMU_CORRECTOR_HPP_
#define IMU_CORRECTOR__IMU_CORRECTOR_HPP_

#include "cxx.h"

#include <sensor_msgs/msg/imu.hpp>

struct ImuCorrectorConfig;

class ImuCorrector
{
public:
  explicit ImuCorrector(const ImuCorrectorConfig &cfg);
  ~ImuCorrector(){}

  sensor_msgs::msg::Imu::ConstSharedPtr get_imu_msg_ptr(){
    return std::make_shared<const sensor_msgs::msg::Imu>(imu_msg_);
  }

  void callbackImu(sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);

private:

  sensor_msgs::msg::Imu imu_msg_;

  double angular_velocity_offset_x_;
  double angular_velocity_offset_y_;
  double angular_velocity_offset_z_;

  double angular_velocity_stddev_xx_;
  double angular_velocity_stddev_yy_;
  double angular_velocity_stddev_zz_;
};

std::unique_ptr<ImuCorrector> new_operator(const ImuCorrectorConfig &cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(ImuCorrector &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);

#endif  // IMU_CORRECTOR__IMU_CORRECTOR_HPP_
