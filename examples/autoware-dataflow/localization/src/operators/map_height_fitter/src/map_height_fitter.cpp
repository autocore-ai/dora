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

#include "map_height_fitter/map_height_fitter.hpp"
#include "map_height_fitter/ffi.rs.h"

#include <time_utils/time_utils.hpp>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <memory>
#include <iostream>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_geometry_msgs.hpp"
#include "cereal_ros_msgs/cereal_sensor_msgs.hpp"

MapHeightFitter::MapHeightFitter(const MapHeightFitterConfig &cfg)
{
  timeout_ = cfg.gnss_pose_timeout;
}

void MapHeightFitter::on_map(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  map_frame_ = msg->header.frame_id;
  map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *map_cloud_);
}

double MapHeightFitter::get_ground_height(const tf2::Vector3 & point) const
{
  constexpr double radius = 1.0 * 1.0;
  const double x = point.getX();
  const double y = point.getY();

  double height = INFINITY;
  for (const auto & p : map_cloud_->points) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    if (sd < radius) {
      height = std::min(height, static_cast<double>(p.z));
    }
  }
  return std::isfinite(height) ? height : point.getZ();
}

void MapHeightFitter::on_fit(
  PoseWithCovarianceStamped::ConstSharedPtr gnss_pose_ptr)
{
  const auto cur_time = std::chrono::system_clock::now();
  const double elapsed = std::chrono::duration_cast<std::chrono::seconds>(
    cur_time - time_utils::from_message(gnss_pose_ptr->header.stamp)).count();
  if (timeout_ < elapsed) {
    const std::string s = "The GNSS pose is out of date.";
    throw std::runtime_error(s);
  }

  const auto & position = gnss_pose_ptr->pose.pose.position;
  tf2::Vector3 point(position.x, position.y, position.z);
  std::string pose_frame = gnss_pose_ptr->header.frame_id;

  if (map_cloud_) {
    // Require that gnss pose frame is consistent with map frame
    if (map_frame_ == pose_frame){
      point.setZ(get_ground_height(point));
    } else {
      std::cerr << "the gnss pose's frame is not consistent with map_frame: " << map_frame_ << std::endl;
    }
  }

  fitted_gnss_pose_ = *gnss_pose_ptr;
  fitted_gnss_pose_.pose.pose.position.x = point.getX();
  fitted_gnss_pose_.pose.pose.position.y = point.getY();
  fitted_gnss_pose_.pose.pose.position.z = point.getZ();
}

std::unique_ptr<MapHeightFitter> new_operator(const MapHeightFitterConfig &cfg)
{
  return std::make_unique<MapHeightFitter>(cfg);
}

OnInputResult on_input(MapHeightFitter &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
  // input process.  
  // std::cout << "MapHeightFitter operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;
  
  // deseralize
  std::stringstream ss; // any(in/out) stream can be used
  copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));
  
  if (id == "map_points"){
    sensor_msgs::msg::PointCloud2::ConstSharedPtr map_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(map_ptr);
    }
    op.on_map(map_ptr);
  } else {
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr gnss_pose_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(gnss_pose_ptr);
    }
    op.on_fit(gnss_pose_ptr);
    // output construct
    ss.clear();
    {
      cereal::PortableBinaryOutputArchive oarchive(ss);
      oarchive(op.get_fitted_gnss_msg_ptr());
    }
    std::string str = ss.str();
    // string --> unsigned char --> rust::Slice 
    auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
    rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
    auto send_result = send_output(output_sender, rust::Str("fitted_gnss_pose"), out_slice);
    OnInputResult result = {send_result.error, false};
    return result;
  }
  // Set the default return
  return OnInputResult{rust::String(""), false};
}
