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

#include "gnss_poser/gnss_poser.hpp"
#include "gnss_poser/ffi.rs.h"

#include "gnss_poser/convert.hpp"

#include <time_utils/time_utils.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_geometry_msgs.hpp"
#include "cereal_ros_msgs/cereal_sensor_msgs.hpp"
#include "cereal_ros_msgs/cereal_autoware_sensing_msgs.hpp"


GNSSPoser::GNSSPoser(const GNSSPoserConfig &cfg)
{
  int coordinate_system = cfg.coordinate_system;
  coordinate_system_ = static_cast<CoordinateSystem>(coordinate_system);

  base_frame_ = static_cast<std::string>(cfg.base_frame);
  gnss_frame_ = static_cast<std::string>(cfg.gnss_antenna_frame);
  gnss_base_frame_ = static_cast<std::string>(cfg.gnss_base_frame);
  map_frame_ = static_cast<std::string>(cfg.map_frame);
  use_gnss_ins_orientation_ = cfg.use_gnss_ins_orientation;
  plane_zone_ = cfg.plane_zone;
  msg_gnss_ins_orientation_stamped_ = std::make_shared<autoware_sensing_msgs::msg::GnssInsOrientationStamped>();

  nav_sat_fix_origin_.latitude = cfg.latitude;
  nav_sat_fix_origin_.longitude = cfg.longitude;
  nav_sat_fix_origin_.altitude = cfg.altitude;

  int buff_epoch = cfg.buff_epoch;
  position_buffer_.set_capacity(buff_epoch);
}

void GNSSPoser::callback_tf_gnssantenna2baselink(
  geometry_msgs::msg::TransformStamped::ConstSharedPtr & TF_msg_ptr)
{
  TF_gnssantenna2baselink_ptr_ = TF_msg_ptr;
}

void GNSSPoser::callbackNavSatFix(
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr)
{
  // check fixed topic
  const bool is_fixed = isFixed(nav_sat_fix_msg_ptr->status);

  // publish is_fixed topic
  is_fixed_msg_.stamp = time_utils::to_message(std::chrono::system_clock::now());
  is_fixed_msg_.data = is_fixed;

  if (!is_fixed) {
    std::cerr << "Not Fixed Topic. Skipping Calculate." << std::endl;
    return;
  }

  // get position in coordinate_system
  const auto gnss_stat = convert(*nav_sat_fix_msg_ptr, coordinate_system_);
  const auto position = getPosition(gnss_stat);

  // calc median position
  position_buffer_.push_front(position);
  if (!position_buffer_.full()) {
    std::cerr << "Buffering Position. Output Skipped." << std::endl;
    return;
  }
  const auto median_position = getMedianPosition(position_buffer_);

  // calc gnss antenna orientation
  geometry_msgs::msg::Quaternion orientation;
  if (use_gnss_ins_orientation_) {
    orientation = msg_gnss_ins_orientation_stamped_->orientation.orientation;
  } else {
    static auto prev_position = median_position;
    orientation = getQuaternionByPositionDifference(median_position, prev_position);
    prev_position = median_position;
  }

  // generate gnss_antenna_pose
  geometry_msgs::msg::Pose gnss_antenna_pose{};
  gnss_antenna_pose.position = median_position;
  gnss_antenna_pose.orientation = orientation;

  // get TF from gnss_antenna to map
  tf2::Transform tf_map2gnss_antenna{};
  tf2::fromMsg(gnss_antenna_pose, tf_map2gnss_antenna);

  // get TF from base_link to gnss_antenna
  tf2::Transform tf_gnss_antenna2base_link{};
  if (!TF_gnssantenna2baselink_ptr_) {
    std::cout << "transform from gnss antenna to baselink is empty, waiting for this message..." << std::endl;
    return;
  } else {
    tf2::fromMsg(TF_gnssantenna2baselink_ptr_->transform, tf_gnss_antenna2base_link);
  }

  // transform pose from gnss_antenna(in map frame) to base_link(in map frame)
  tf2::Transform tf_map2base_link{};
  tf_map2base_link = tf_map2gnss_antenna * tf_gnss_antenna2base_link;

  gnss_base_pose_msg_.header.stamp = nav_sat_fix_msg_ptr->header.stamp;
  gnss_base_pose_msg_.header.frame_id = map_frame_;
  tf2::toMsg(tf_map2base_link, gnss_base_pose_msg_.pose);

  // publish gnss_base_link pose in map frame
  double gnss_x_bias = 687072.78; // bias of map origin
  double gnss_y_bias = 3510747.06; // bias of map oirgin
  gnss_base_pose_msg_.pose.position.x -= gnss_x_bias;
  gnss_base_pose_msg_.pose.position.y -= gnss_y_bias;

  // publish gnss_base_link pose_cov in map frame
  gnss_base_pose_cov_msg_.header = gnss_base_pose_msg_.header;
  gnss_base_pose_cov_msg_.pose.pose = gnss_base_pose_msg_.pose;
  gnss_base_pose_cov_msg_.pose.covariance[7 * 0] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[0] : 10.0;
  gnss_base_pose_cov_msg_.pose.covariance[7 * 1] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[4] : 10.0;
  gnss_base_pose_cov_msg_.pose.covariance[7 * 2] =
    canGetCovariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[8] : 10.0;

  if (use_gnss_ins_orientation_) {
    gnss_base_pose_cov_msg_.pose.covariance[7 * 3] =
      std::pow(msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_x, 2);
    gnss_base_pose_cov_msg_.pose.covariance[7 * 4] =
      std::pow(msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_y, 2);
    gnss_base_pose_cov_msg_.pose.covariance[7 * 5] =
      std::pow(msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_z, 2);
  } else {
    gnss_base_pose_cov_msg_.pose.covariance[7 * 3] = 0.1;
    gnss_base_pose_cov_msg_.pose.covariance[7 * 4] = 0.1;
    gnss_base_pose_cov_msg_.pose.covariance[7 * 5] = 1.0;
  }

  // set map to gnss_base_link
  setTF(map_frame_, gnss_base_frame_, gnss_base_pose_msg_);
}

void GNSSPoser::callbackGnssInsOrientationStamped(
  const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr msg)
{
  *msg_gnss_ins_orientation_stamped_ = *msg;
}

bool GNSSPoser::isFixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg)
{
  return nav_sat_status_msg.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

bool GNSSPoser::canGetCovariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg)
{
  return nav_sat_fix_msg.position_covariance_type >
         sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

GNSSStat GNSSPoser::convert(
  const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg, CoordinateSystem coordinate_system)
{
  GNSSStat gnss_stat;
  if (coordinate_system == CoordinateSystem::UTM) {
    gnss_stat = NavSatFix2UTM(nav_sat_fix_msg);
  } else if (coordinate_system == CoordinateSystem::LOCAL_CARTESIAN_UTM) {
    gnss_stat =
      NavSatFix2LocalCartesianUTM(nav_sat_fix_msg, nav_sat_fix_origin_);
  } else if (coordinate_system == CoordinateSystem::MGRS) {
    gnss_stat = NavSatFix2MGRS(nav_sat_fix_msg, MGRSPrecision::_100MICRO_METER);
  } else if (coordinate_system == CoordinateSystem::PLANE) {
    gnss_stat = NavSatFix2PLANE(nav_sat_fix_msg, plane_zone_);
  } else if (coordinate_system == CoordinateSystem::LOCAL_CARTESIAN_WGS84) {
    gnss_stat =
      NavSatFix2LocalCartesianWGS84(nav_sat_fix_msg, nav_sat_fix_origin_);
  } else {
    std::cerr << "Unknown Coordinate System" << std::endl;
  }
  return gnss_stat;
}

geometry_msgs::msg::Point GNSSPoser::getPosition(const GNSSStat & gnss_stat)
{
  geometry_msgs::msg::Point point;
  point.x = gnss_stat.x;
  point.y = gnss_stat.y;
  point.z = gnss_stat.z;
  return point;
}

geometry_msgs::msg::Point GNSSPoser::getMedianPosition(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer)
{
  auto getMedian = [](std::vector<double> array) {
    std::sort(std::begin(array), std::end(array));
    const size_t median_index = array.size() / 2;
    double median = (array.size() % 2)
                      ? (array.at(median_index))
                      : ((array.at(median_index) + array.at(median_index - 1)) / 2);
    return median;
  };

  std::vector<double> array_x;
  std::vector<double> array_y;
  std::vector<double> array_z;
  for (const auto & position : position_buffer) {
    array_x.push_back(position.x);
    array_y.push_back(position.y);
    array_z.push_back(position.z);
  }

  geometry_msgs::msg::Point median_point;
  median_point.x = getMedian(array_x);
  median_point.y = getMedian(array_y);
  median_point.z = getMedian(array_z);
  return median_point;
}

geometry_msgs::msg::Quaternion GNSSPoser::getQuaternionByHeading(const int heading)
{
  int heading_conv = 0;
  // convert heading[0(North)~360] to yaw[-M_PI(West)~M_PI]
  if (heading >= 0 && heading <= 27000000) {
    heading_conv = 9000000 - heading;
  } else {
    heading_conv = 45000000 - heading;
  }
  const double yaw = (heading_conv * 1e-5) * M_PI / 180.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);

  return tf2::toMsg(quaternion);
}

geometry_msgs::msg::Quaternion GNSSPoser::getQuaternionByPositionDifference(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point)
{
  const double yaw = std::atan2(point.y - prev_point.y, point.x - prev_point.x);
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);
  return tf2::toMsg(quaternion);
}

void GNSSPoser::setTF(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::msg::PoseStamped & pose_msg)
{
  TF_map2gnssbase_msg_.header.frame_id = frame_id;
  TF_map2gnssbase_msg_.child_frame_id = child_frame_id;
  TF_map2gnssbase_msg_.header.stamp = pose_msg.header.stamp;

  TF_map2gnssbase_msg_.transform.translation.x = pose_msg.pose.position.x;
  TF_map2gnssbase_msg_.transform.translation.y = pose_msg.pose.position.y;
  TF_map2gnssbase_msg_.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  TF_map2gnssbase_msg_.transform.rotation.x = tf_quaternion.x();
  TF_map2gnssbase_msg_.transform.rotation.y = tf_quaternion.y();
  TF_map2gnssbase_msg_.transform.rotation.z = tf_quaternion.z();
  TF_map2gnssbase_msg_.transform.rotation.w = tf_quaternion.w();
}


std::unique_ptr<GNSSPoser> new_operator(const GNSSPoserConfig &cfg)
{
  return std::make_unique<GNSSPoser>(cfg);
}

OnInputResult on_input(GNSSPoser &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
  // input process.  
  // std::cout << "GNSSPoser operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;
  
  // deseralize
  std::stringstream ss; // any(in/out) stream can be used
  copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));

  if (id == "orientation") {
    autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr orientation_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss); // Create an input archive
      iarchive(orientation_ptr); // Read the data from the archive
    }
    op.callbackGnssInsOrientationStamped(orientation_ptr);
  } else if (id == "tf_gnssantenna2baselink") {
    geometry_msgs::msg::TransformStamped::ConstSharedPtr TF_gnssantenna2baselink_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(TF_gnssantenna2baselink_ptr);
    }
    op.callback_tf_gnssantenna2baselink(TF_gnssantenna2baselink_ptr);
  } else {
    sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(nav_sat_fix_ptr);
    }
    op.callbackNavSatFix(nav_sat_fix_ptr);

    // output construct
    ss.clear();
    {
      cereal::PortableBinaryOutputArchive oarchive(ss); // Create an output archive
      oarchive(op.get_gnss_pose_msg_ptr()); // Write the data to the archive
    }
    std::string str = ss.str();
    auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
    rust::Slice<const uint8_t> out_slice_pose{uchar.data(), uchar.size()};
    auto send_result_pose = send_output(output_sender, rust::Str("gnss_pose_cov"), out_slice_pose);
    OnInputResult result_pose = {send_result_pose.error, false};
    return result_pose;
  }
  // Set the default return
  return OnInputResult{rust::String(""), false};
}

