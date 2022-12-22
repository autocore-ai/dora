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
#ifndef GNSS_POSER__GNSS_POSER_HPP_
#define GNSS_POSER__GNSS_POSER_HPP_

#include "cxx.h"

#include "gnss_poser/gnss_stat.hpp"

#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tier4_debug_msgs/msg/bool_stamped.hpp>

#include <boost/circular_buffer.hpp>

#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>

struct GNSSPoserConfig;

class GNSSPoser
{
public:
  explicit GNSSPoser(const GNSSPoserConfig &cfg);
  ~GNSSPoser(){}

  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr get_gnss_pose_msg_ptr(){
    return std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(gnss_base_pose_cov_msg_);
  }

  void callbackNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr);
  void callbackGnssInsOrientationStamped(
    const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr msg);
  void callback_tf_gnssantenna2baselink(
    geometry_msgs::msg::TransformStamped::ConstSharedPtr & TF_msg_ptr);

private:
  bool isFixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg);
  bool canGetCovariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg);
  GNSSStat convert(
    const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, CoordinateSystem coordinate_system);
  geometry_msgs::msg::Point getPosition(const GNSSStat & gnss_stat);
  geometry_msgs::msg::Point getMedianPosition(
    const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer);
  geometry_msgs::msg::Quaternion getQuaternionByHeading(const int heading);
  geometry_msgs::msg::Quaternion getQuaternionByPositionDifference(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point);

  void setTF(
    const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::msg::PoseStamped & pose_msg);

  geometry_msgs::msg::PoseStamped gnss_base_pose_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped gnss_base_pose_cov_msg_;
  tier4_debug_msgs::msg::BoolStamped is_fixed_msg_;

  geometry_msgs::msg::TransformStamped::ConstSharedPtr TF_gnssantenna2baselink_ptr_;
  geometry_msgs::msg::TransformStamped TF_map2gnssbase_msg_;

  CoordinateSystem coordinate_system_;
  std::string base_frame_;
  std::string gnss_frame_;
  std::string gnss_base_frame_;
  std::string map_frame_;

  sensor_msgs::msg::NavSatFix nav_sat_fix_origin_;
  bool use_gnss_ins_orientation_;

  boost::circular_buffer<geometry_msgs::msg::Point> position_buffer_;

  int plane_zone_;

  autoware_sensing_msgs::msg::GnssInsOrientationStamped::SharedPtr
    msg_gnss_ins_orientation_stamped_;
};

std::unique_ptr<GNSSPoser> new_operator(const GNSSPoserConfig &cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(GNSSPoser &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);

#endif  // GNSS_POSER__GNSS_POSER_HPP_
