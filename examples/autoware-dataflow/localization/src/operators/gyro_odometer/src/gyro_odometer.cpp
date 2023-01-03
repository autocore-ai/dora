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

#include "gyro_odometer/gyro_odometer.hpp"
#include "gyro_odometer/ffi.rs.h"

#include <time_utils/time_utils.hpp>

#include <iostream>
#include <cmath>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_sensor_msgs.hpp"
#include "cereal_ros_msgs/cereal_geometry_msgs.hpp"


GyroOdometer::GyroOdometer(const GyroOdometerConfig &cfg)
{
  output_frame_ = static_cast<std::string>(cfg.output_frame);
  message_timeout_sec_ = cfg.msg_timeout_sec;
}

void GyroOdometer::callbackTwistWithCovariance(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_with_cov_msg_ptr)
{
  // TODO(YamatoAndo) trans from twist_with_cov_msg_ptr->header to base_frame
  twist_with_cov_msg_ptr_ = twist_with_cov_msg_ptr;

  /*
  if (!imu_msg_ptr_) {
    std::cerr << "Imu msg is not subscribed" << std::endl;
    return;
  }

  const double imu_dt = std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - time_utils::from_message(imu_msg_ptr_->header.stamp)).count()) / 1000.0;
  if (imu_dt > message_timeout_sec_) {
    std::cerr << "Imu msg is timeout. imu_dt: "<< imu_dt << "[sec], tolerance " << message_timeout_sec_ << "[sec]" << std::endl;
    return;
  }
  */
}

void GyroOdometer::callback_tf_baselink2imu(geometry_msgs::msg::TransformStamped::ConstSharedPtr & TF_msg_ptr)
{
    TF_baselink2imu_ptr_ = TF_msg_ptr;
}

void GyroOdometer::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  imu_msg_ptr_ = imu_msg_ptr;
  if (!twist_with_cov_msg_ptr_) {
    std::cerr << "Twist msg is not subscribed" << std::endl;
    return;
  }

  /*
  const double twist_dt = std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now() - time_utils::from_message(twist_with_cov_msg_ptr_->header.stamp)).count()) / 1000.0;
  if (twist_dt > message_timeout_sec_) {
    std::cerr << "Twist msg is timeout. twist_dt: "<< twist_dt << "[sec], tolerance " << message_timeout_sec_ << "[sec]" << std::endl;
    return;
  }
  */

  if (!TF_baselink2imu_ptr_) {
    std::cout << "transform from baselink to imu is empty, waiting for this message..." << std::endl;
    return;
  }

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.header = imu_msg_ptr_->header;
  angular_velocity.vector = imu_msg_ptr_->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  transformed_angular_velocity.header = TF_baselink2imu_ptr_->header;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *TF_baselink2imu_ptr_);

  // TODO(YamatoAndo) move code
  twist_msg_.header.stamp = imu_msg_ptr_->header.stamp;
  twist_msg_.header.frame_id = output_frame_;
  twist_msg_.twist.linear = twist_with_cov_msg_ptr_->twist.twist.linear;
  twist_msg_.twist.angular.x = transformed_angular_velocity.vector.x;
  twist_msg_.twist.angular.y = transformed_angular_velocity.vector.y;
  twist_msg_.twist.angular.z = transformed_angular_velocity.vector.z;

  twist_with_cov_msg_.header.stamp = imu_msg_ptr_->header.stamp;
  twist_with_cov_msg_.header.frame_id = output_frame_;
  twist_with_cov_msg_.twist.twist.linear = twist_with_cov_msg_ptr_->twist.twist.linear;
  twist_with_cov_msg_.twist.twist.angular.x = transformed_angular_velocity.vector.x;
  twist_with_cov_msg_.twist.twist.angular.y = transformed_angular_velocity.vector.y;
  twist_with_cov_msg_.twist.twist.angular.z = transformed_angular_velocity.vector.z;

  // NOTE
  // linear.y and linear.z are not measured values.
  // Therefore, they should be assigned large variance values.
  twist_with_cov_msg_.twist.covariance[0] = twist_with_cov_msg_ptr_->twist.covariance[0];
  twist_with_cov_msg_.twist.covariance[7] = 10000.0;
  twist_with_cov_msg_.twist.covariance[14] = 10000.0;
  twist_with_cov_msg_.twist.covariance[21] = imu_msg_ptr_->angular_velocity_covariance[0];
  twist_with_cov_msg_.twist.covariance[28] = imu_msg_ptr_->angular_velocity_covariance[4];
  twist_with_cov_msg_.twist.covariance[35] = imu_msg_ptr_->angular_velocity_covariance[8];

  // clear imu yaw bias if vehicle is stopped
  if (
    std::fabs(transformed_angular_velocity.vector.z) < 0.01 &&
    std::fabs(twist_with_cov_msg_ptr_->twist.twist.linear.x) < 0.01) {
    twist_msg_.twist.angular.x = 0.0;
    twist_msg_.twist.angular.y = 0.0;
    twist_msg_.twist.angular.z = 0.0;
    twist_with_cov_msg_.twist.twist.angular.x = 0.0;
    twist_with_cov_msg_.twist.twist.angular.y = 0.0;
    twist_with_cov_msg_.twist.twist.angular.z = 0.0;
  }
}


std::unique_ptr<GyroOdometer> new_operator(const GyroOdometerConfig &cfg)
{
  return std::make_unique<GyroOdometer>(cfg);
}

OnInputResult on_input(GyroOdometer &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
  // input process.  
  // std::cout << "GyroOdometer operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;
  
  // deseralize
  std::stringstream ss; // any(in/out) stream can be used
  copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));
  if (id == "tf_baselink2imu") {
    geometry_msgs::msg::TransformStamped::ConstSharedPtr TF_baselink2imu_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(TF_baselink2imu_ptr);
    }
    op.callback_tf_baselink2imu(TF_baselink2imu_ptr);
  } else if (id == "vehicle_twist") {
    geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_with_cov_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(twist_with_cov_ptr);
    }
    op.callbackTwistWithCovariance(twist_with_cov_ptr);
  } else {
    sensor_msgs::msg::Imu::ConstSharedPtr imu_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(imu_ptr);
    }
    op.callbackImu(imu_ptr);

    // output construct
    ss.clear();
    {
      cereal::PortableBinaryOutputArchive oarchive(ss);
      oarchive(op.get_twist_with_cov_msg_ptr());
    }
    std::string str = ss.str();
    // string --> unsigned char --> rust::Slice 
    auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
    rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
    auto send_result = send_output(output_sender, rust::Str("twist_with_cov"), out_slice);
    OnInputResult result = {send_result.error, false};
    return result;
  }
  // Set the default return, which indicates there are two possibilities.
  // First: receive an input among [init_pose, map, tf_baselink2lidar].
  // Second: receive the pointcloud input, but ndt is not converged.
  return OnInputResult{rust::String(""), false};
}
