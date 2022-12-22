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

#ifndef NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_
#define NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_

#define FMT_HEADER_ONLY

#include "cxx.h"

#include "ndt_scan_matcher/particle.hpp"

#include <ndt/omp.hpp>
#include <ndt/pcl_generic.hpp>
#include <ndt/pcl_modified.hpp>
#include <rclcpp/rclcpp.hpp>

// #include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fmt/format.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct NDTScanMatcherConfig;

enum class NDTImplementType { PCL_GENERIC = 0, PCL_MODIFIED = 1, OMP = 2 };
enum class ConvergedParamType {
  TRANSFORM_PROBABILITY = 0,
  NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD = 1
};

template <typename PointSource, typename PointTarget>
std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> getNDT(
  const NDTImplementType & ndt_mode)
{
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr;
  if (ndt_mode == NDTImplementType::PCL_GENERIC) {
    ndt_ptr.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_mode == NDTImplementType::PCL_MODIFIED) {
    ndt_ptr.reset(new NormalDistributionsTransformPCLModified<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_mode == NDTImplementType::OMP) {
    ndt_ptr.reset(new NormalDistributionsTransformOMP<PointSource, PointTarget>);
    return ndt_ptr;
  }

  const std::string s = fmt::format("Unknown NDT type {}", static_cast<int>(ndt_mode));
  throw std::runtime_error(s);
}

class NDTScanMatcher
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;

  // TODO(Tier IV): move file
  struct OMPParams
  {
    OMPParams() : search_method(pclomp::NeighborSearchMethod::KDTREE), num_threads(1) {}
    pclomp::NeighborSearchMethod search_method;
    int num_threads;
  };

public:
  NDTScanMatcher(const NDTScanMatcherConfig &cfg);
  ~NDTScanMatcher(){}

  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr get_ndt_pose_msg_ptr(){
    // return std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(ndt_pose_with_cov_msg_);
    return std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(ndt_pose_with_cov_msg_);
  }
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr get_aligned_response_msg_ptr(){
    return std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(aligned_response_msg_);
  }
  bool get_is_converged(){return is_converged;}
  bool get_is_align_succeeded(){return is_align_succeeded_;}

  void callbackMapPoints(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void callbackSensorPoints(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void serviceNDTAlign(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr init_pose_msg_ptr);
  void callbackInitialPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);
  void callbackRegularizationPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);
  void callback_tf_baselink2lidar(
    geometry_msgs::msg::TransformStamped::ConstSharedPtr & TF_msg_ptr);
  
private:
  void setTF(
    const std::string & child_frame_id, const geometry_msgs::msg::PoseStamped & pose_msg);
  
  geometry_msgs::msg::PoseWithCovarianceStamped alignUsingMonteCarlo(
    const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov);

  bool validateTimeStampDifference(
    const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
    const double time_tolerance_sec);
  bool validatePositionDifference(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & reference_point, const double distance_tolerance_m_);

  std::optional<Eigen::Matrix4f> interpolateRegularizationPose(
    const rclcpp::Time & sensor_ros_time);

  geometry_msgs::msg::TransformStamped::ConstSharedPtr TF_baselink2lidar_ptr_;
  geometry_msgs::msg::TransformStamped TF_map2ndtbase_msg_;
  bool is_converged = false;

  sensor_msgs::msg::PointCloud2 sensor_aligned_pose_msg_;
  geometry_msgs::msg::PoseStamped ndt_pose_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped ndt_pose_with_cov_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_with_cov_msg_;

  tier4_debug_msgs::msg::Float32Stamped exe_time_msg_;
  tier4_debug_msgs::msg::Float32Stamped transform_probability_msg_;
  tier4_debug_msgs::msg::Float32Stamped nearest_voxel_transformation_likelihood_msg_;
  tier4_debug_msgs::msg::Int32Stamped iteration_num_msg_;
  tier4_debug_msgs::msg::Float32Stamped initial_to_result_distance_msg_;
  tier4_debug_msgs::msg::Float32Stamped initial_to_result_distance_old_msg_;
  tier4_debug_msgs::msg::Float32Stamped initial_to_result_distance_new_msg_;

  visualization_msgs::msg::MarkerArray ndt_marker_msg_;

  // diagnostic_msgs::msg::DiagnosticArray diagnostics_msg_;
  visualization_msgs::msg::MarkerArray ndt_monte_carlo_initial_pose_marker_msg_;
  bool is_align_succeeded_ = false;
  geometry_msgs::msg::PoseWithCovarianceStamped aligned_response_msg_;

  NDTImplementType ndt_implement_type_;
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr_;

  Eigen::Matrix4f base_to_sensor_matrix_;
  std::string base_frame_;
  std::string ndt_base_frame_;
  std::string map_frame_;

  ConvergedParamType converged_param_type_;
  double converged_param_transform_probability_;
  double converged_param_nearest_voxel_transformation_likelihood_;

  int initial_estimate_particles_num_;
  double initial_pose_timeout_sec_;
  double initial_pose_distance_tolerance_m_;
  float inversion_vector_threshold_;
  float oscillation_threshold_;
  std::array<double, 36> output_pose_covariance_;

  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    initial_pose_msg_ptr_array_;
  std::mutex ndt_map_mtx_;
  std::mutex initial_pose_array_mtx_;

  OMPParams omp_params_;

  // std::thread diagnostic_thread_;
  std::map<std::string, std::string> key_value_stdmap_;

  bool regularization_enabled_;
  float regularization_scale_factor_;
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    regularization_pose_msg_ptr_array_;
};

std::unique_ptr<NDTScanMatcher> new_operator(const NDTScanMatcherConfig &cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(NDTScanMatcher &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);

#endif  // NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_HPP_
