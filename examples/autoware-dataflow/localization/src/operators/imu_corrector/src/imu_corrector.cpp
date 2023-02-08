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

#include "imu_corrector/imu_corrector.hpp"
#include "imu_corrector/ffi.rs.h"

#include <memory>
#include <iostream>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_sensor_msgs.hpp"

ImuCorrector::ImuCorrector(const ImuCorrectorConfig &cfg)
{
  angular_velocity_offset_x_ = cfg.angular_velocity_offset_x;
  angular_velocity_offset_y_ = cfg.angular_velocity_offset_y;
  angular_velocity_offset_z_ = cfg.angular_velocity_offset_z;

  angular_velocity_stddev_xx_ = cfg.angular_velocity_stddev_xx;
  angular_velocity_stddev_yy_ = cfg.angular_velocity_stddev_yy;
  angular_velocity_stddev_zz_ = cfg.angular_velocity_stddev_zz;
}

void ImuCorrector::callbackImu(sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  imu_msg_ = *imu_msg_ptr;

  imu_msg_.angular_velocity.x -= angular_velocity_offset_x_;
  imu_msg_.angular_velocity.y -= angular_velocity_offset_y_;
  imu_msg_.angular_velocity.z -= angular_velocity_offset_z_;

  imu_msg_.angular_velocity_covariance[0 * 3 + 0] =
    angular_velocity_stddev_xx_ * angular_velocity_stddev_xx_;
  imu_msg_.angular_velocity_covariance[1 * 3 + 1] =
    angular_velocity_stddev_yy_ * angular_velocity_stddev_yy_;
  imu_msg_.angular_velocity_covariance[2 * 3 + 2] =
    angular_velocity_stddev_zz_ * angular_velocity_stddev_zz_;
}


std::unique_ptr<ImuCorrector> new_operator(const ImuCorrectorConfig &cfg)
{
  return std::make_unique<ImuCorrector>(cfg);
}

OnInputResult on_input(ImuCorrector &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
  // input process.  
  // std::cout << "ImuCorrector operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;
  
  // deseralize
  std::stringstream ss; // any(in/out) stream can be used
  copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));
  
  sensor_msgs::msg::Imu::ConstSharedPtr imu_ptr;
  {
    cereal::PortableBinaryInputArchive iarchive(ss);
    iarchive(imu_ptr);
  }
  op.callbackImu(imu_ptr);

  // output construct
  ss.str(""); // clear the buffer of ss
  {
    cereal::PortableBinaryOutputArchive oarchive(ss);
    oarchive(op.get_imu_msg_ptr());
  }
  std::string str = ss.str();
  // string --> unsigned char --> rust::Slice 
  auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
  rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
  auto send_result = send_output(output_sender, rust::Str("imu_data"), out_slice);
  OnInputResult result = {send_result.error, false};
  return result;
}

