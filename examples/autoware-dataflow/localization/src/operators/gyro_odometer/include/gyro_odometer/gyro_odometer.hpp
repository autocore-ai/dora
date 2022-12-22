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

// Copyright 2015-2019 Autoware Foundation
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

#ifndef GYRO_ODOMETER__GYRO_ODOMETER_HPP_
#define GYRO_ODOMETER__GYRO_ODOMETER_HPP_

#include "cxx.h"

#include <memory>
#include <vector>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2/transform_datatypes.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

struct GyroOdometerConfig;

class GyroOdometer
{
public:
  GyroOdometer(const GyroOdometerConfig &cfg);
  ~GyroOdometer(){}

  geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr get_twist_with_cov_msg_ptr(){
    // return std::make_shared<const geometry_msgs::msg::TwistWithCovarianceStamped>(twist_with_cov_msg_);
    return std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>(twist_with_cov_msg_);
  }

  void callbackTwistWithCovariance(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_with_cov_msg_ptr);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);
  void callback_tf_baselink2imu(geometry_msgs::msg::TransformStamped::ConstSharedPtr & TF_msg_ptr);

private:
  geometry_msgs::msg::TwistStamped twist_msg_;
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov_msg_;

  std::string output_frame_;
  double message_timeout_sec_;

  geometry_msgs::msg::TransformStamped::ConstSharedPtr TF_baselink2imu_ptr_;

  geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_with_cov_msg_ptr_;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr_;
};


std::unique_ptr<GyroOdometer> new_operator(const GyroOdometerConfig &cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(GyroOdometer &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);

#endif  // GYRO_ODOMETER__GYRO_ODOMETER_HPP_
