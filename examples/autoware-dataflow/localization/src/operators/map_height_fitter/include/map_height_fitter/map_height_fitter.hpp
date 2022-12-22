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

#ifndef MAP_HEIGHT_FITTER__MAP_HEIGHT_FITTER_HPP_
#define MAP_HEIGHT_FITTER__MAP_HEIGHT_FITTER_HPP_

#include "cxx.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/LinearMath/Vector3.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

struct MapHeightFitterConfig;

class MapHeightFitter
{

  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  MapHeightFitter(const MapHeightFitterConfig &cfg);
  ~MapHeightFitter(){}

  PoseWithCovarianceStamped::ConstSharedPtr get_fitted_gnss_msg_ptr(){
    return std::make_shared<const PoseWithCovarianceStamped>(fitted_gnss_pose_);
  }

  void on_map(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void on_fit(PoseWithCovarianceStamped::ConstSharedPtr gnss_pose_ptr);
  
private:

  std::string map_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  PoseWithCovarianceStamped fitted_gnss_pose_;

  double timeout_;

  double get_ground_height(const tf2::Vector3 & point) const;
};

std::unique_ptr<MapHeightFitter> new_operator(const MapHeightFitterConfig &cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(MapHeightFitter &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);

#endif  // MAP_HEIGHT_FITTER__MAP_HEIGHT_FITTER_HPP_
