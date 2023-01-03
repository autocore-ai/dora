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

#include "ndt_scan_matcher/ndt_scan_matcher.hpp"
#include "ndt_scan_matcher/ffi.rs.h"

#include "ndt_scan_matcher/debug.hpp"
#include "ndt_scan_matcher/matrix_type.hpp"
#include "ndt_scan_matcher/particle.hpp"
#include "ndt_scan_matcher/util_func.hpp"

#include <time_utils/time_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>

#include <iostream>
// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_sensor_msgs.hpp"
#include "cereal_ros_msgs/cereal_geometry_msgs.hpp"


tier4_debug_msgs::msg::Float32Stamped makeFloat32Stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = tier4_debug_msgs::msg::Float32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

tier4_debug_msgs::msg::Int32Stamped makeInt32Stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = tier4_debug_msgs::msg::Int32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

geometry_msgs::msg::TransformStamped identityTransformStamped(
  const builtin_interfaces::msg::Time & timestamp, const std::string & header_frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = timestamp;
  transform.header.frame_id = header_frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform.rotation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
  transform.transform.translation = tier4_autoware_utils::createTranslation(0.0, 0.0, 0.0);
  return transform;
}

double norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::sqrt(
    std::pow(p1.x - p2.x, 2.0) + std::pow(p1.y - p2.y, 2.0) + std::pow(p1.z - p2.z, 2.0));
}

bool isLocalOptimalSolutionOscillation(
  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &
    result_pose_matrix_array,
  const float oscillation_threshold, const float inversion_vector_threshold)
{
  bool prev_oscillation = false;
  int oscillation_cnt = 0;
  for (size_t i = 2; i < result_pose_matrix_array.size(); ++i) {
    const Eigen::Vector3f current_pose = result_pose_matrix_array.at(i).block(0, 3, 3, 1);
    const Eigen::Vector3f prev_pose = result_pose_matrix_array.at(i - 1).block(0, 3, 3, 1);
    const Eigen::Vector3f prev_prev_pose = result_pose_matrix_array.at(i - 2).block(0, 3, 3, 1);
    const auto current_vec = (current_pose - prev_pose).normalized();
    const auto prev_vec = (prev_pose - prev_prev_pose).normalized();
    const bool oscillation = prev_vec.dot(current_vec) < inversion_vector_threshold;
    if (prev_oscillation && oscillation) {
      if (oscillation_cnt > oscillation_threshold) {
        return true;
      }
      ++oscillation_cnt;
    } else {
      oscillation_cnt = 0;
    }
    prev_oscillation = oscillation;
  }
  return false;
}

NDTScanMatcher::NDTScanMatcher(const NDTScanMatcherConfig &cfg)
{
  key_value_stdmap_["state"] = "Initializing";

  int ndt_implement_type_tmp = cfg.ndt_implement_type;
  ndt_implement_type_ = static_cast<NDTImplementType>(ndt_implement_type_tmp);

  std::cout << "NDT Implement Type is " << ndt_implement_type_tmp << std::endl;
  try {
    ndt_ptr_ = getNDT<PointSource, PointTarget>(ndt_implement_type_);
  } catch (std::exception & e) {
    std::cerr << "NDT initialization exception: " << e.what() << std::endl;
    return;
  }

  if (ndt_implement_type_ == NDTImplementType::OMP) {
    using T = NormalDistributionsTransformOMP<PointSource, PointTarget>;

    // FIXME(IshitaTakeshi) Not sure if this is safe
    std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr_);
    int search_method = static_cast<int>(omp_params_.search_method);
    search_method = cfg.omp_neighborhood_search_method;
    omp_params_.search_method = static_cast<pclomp::NeighborSearchMethod>(search_method);
    // TODO(Tier IV): check search_method is valid value.
    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);

    omp_params_.num_threads = cfg.omp_num_threads;
    omp_params_.num_threads = std::max(omp_params_.num_threads, 1);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);
    ndt_ptr_ = ndt_omp_ptr;
  }

  base_frame_ = static_cast<std::string>(cfg.base_frame);
  ndt_base_frame_ = "ndt_base_frame";
  map_frame_ = "map";
  regularization_enabled_ = cfg.regularization_enabled;
  regularization_scale_factor_ = cfg.regularization_scale_factor;

  double trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  double step_size = ndt_ptr_->getStepSize();
  double resolution = ndt_ptr_->getResolution();
  int max_iterations = ndt_ptr_->getMaximumIterations();
  trans_epsilon = cfg.trans_epsilon;
  step_size = cfg.step_size;
  resolution = cfg.resolution;
  max_iterations = cfg.max_iterations;
  ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  ndt_ptr_->setStepSize(step_size);
  ndt_ptr_->setResolution(resolution);
  ndt_ptr_->setMaximumIterations(max_iterations);
  ndt_ptr_->setRegularizationScaleFactor(regularization_scale_factor_);

  int converged_param_type_tmp = cfg.converged_param_type;
  converged_param_type_ = static_cast<ConvergedParamType>(converged_param_type_tmp);
  if (
    ndt_implement_type_ != NDTImplementType::OMP &&
    converged_param_type_ == ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    std::cerr << "ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD is only available"
      "when NDTImplementType::OMP is selected." << std::endl;
    return;
  }

  converged_param_transform_probability_ = cfg.converged_param_transform_probability;
  converged_param_nearest_voxel_transformation_likelihood_ = cfg.converged_param_nearest_voxel_transformation_likelihood;

  initial_estimate_particles_num_ = cfg.initial_estimate_particles_num;
  initial_pose_timeout_sec_ = cfg.initial_pose_timeout_sec;
  initial_pose_distance_tolerance_m_ = cfg.initial_pose_distance_tolerance_m;
  inversion_vector_threshold_ = -0.9;
  oscillation_threshold_ = 10;

  output_pose_covariance_ = cfg.output_pose_covariance;
}

void NDTScanMatcher::serviceNDTAlign(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr init_pose_msg_ptr)
{
  /*
  if (init_pose_msg_ptr->header.frame_id == map_frame_) {
    const auto mapTF_initial_pose_msg = *init_pose_msg_ptr;
  } else {
    // get TF from pose_frame to map_frame
    auto TF_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
    getTransform(map_frame_, init_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    const auto mapTF_initial_pose_msg = transform(*init_pose_msg_ptr, *TF_pose_to_map_ptr);
  }
  */
  const auto mapTF_initial_pose_msg = *init_pose_msg_ptr;

  if (ndt_ptr_->getInputTarget() == nullptr) {
    is_align_succeeded_ = false;
    const std::string s = "No InputTarget, NDT align server failed";
    throw std::runtime_error(s);
  }
  if (ndt_ptr_->getInputSource() == nullptr) {
    is_align_succeeded_ = false;
    const std::string s = "No InputSource, NDT align server failed";
    throw std::runtime_error(s);
  }

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  key_value_stdmap_["state"] = "Aligning";
  aligned_response_msg_ = alignUsingMonteCarlo(ndt_ptr_, mapTF_initial_pose_msg);
  key_value_stdmap_["state"] = "Sleeping";
  is_align_succeeded_ = true;
  aligned_response_msg_.pose.covariance = init_pose_msg_ptr->pose.covariance;
}

void NDTScanMatcher::callbackInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  // lock mutex for initial pose
  std::lock_guard<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);
  // if rosbag restart, clear buffer
  if (!initial_pose_msg_ptr_array_.empty()) {
    const builtin_interfaces::msg::Time & t_front =
      initial_pose_msg_ptr_array_.front()->header.stamp;
    const builtin_interfaces::msg::Time & t_msg = initial_pose_msg_ptr->header.stamp;
    if (t_front.sec > t_msg.sec || (t_front.sec == t_msg.sec && t_front.nanosec > t_msg.nanosec)) {
      initial_pose_msg_ptr_array_.clear();
    }
  }

  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_msg_ptr_array_.push_back(initial_pose_msg_ptr);
  } else {
    // // get TF from pose_frame to map_frame
    // auto TF_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
    // getTransform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // // transform pose_frame to map_frame
    // auto mapTF_initial_pose_msg_ptr =
    //   std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    // *mapTF_initial_pose_msg_ptr = transform(*initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    // initial_pose_msg_ptr_array_.push_back(mapTF_initial_pose_msg_ptr);
  }
}

void NDTScanMatcher::callbackRegularizationPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  regularization_pose_msg_ptr_array_.push_back(pose_conv_msg_ptr);
}

void NDTScanMatcher::callbackMapPoints(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_msg_ptr)
{
  const auto trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  const auto step_size = ndt_ptr_->getStepSize();
  const auto resolution = ndt_ptr_->getResolution();
  const auto max_iterations = ndt_ptr_->getMaximumIterations();

  using NDTBase = NormalDistributionsTransformBase<PointSource, PointTarget>;
  std::shared_ptr<NDTBase> new_ndt_ptr = getNDT<PointSource, PointTarget>(ndt_implement_type_);

  if (ndt_implement_type_ == NDTImplementType::OMP) {
    using T = NormalDistributionsTransformOMP<PointSource, PointTarget>;

    // FIXME(IshitaTakeshi) Not sure if this is safe
    std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr_);
    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);
    new_ndt_ptr = ndt_omp_ptr;
  }

  new_ndt_ptr->setTransformationEpsilon(trans_epsilon);
  new_ndt_ptr->setStepSize(step_size);
  new_ndt_ptr->setResolution(resolution);
  new_ndt_ptr->setMaximumIterations(max_iterations);
  new_ndt_ptr->setRegularizationScaleFactor(regularization_scale_factor_);

  pcl::shared_ptr<pcl::PointCloud<PointTarget>> map_points_ptr(new pcl::PointCloud<PointTarget>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  new_ndt_ptr->setInputTarget(map_points_ptr);
  // create Thread
  // detach
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  new_ndt_ptr->align(*output_cloud, Eigen::Matrix4f::Identity());

  // swap
  ndt_map_mtx_.lock();
  ndt_ptr_ = new_ndt_ptr;
  ndt_map_mtx_.unlock();
}

void NDTScanMatcher::callback_tf_baselink2lidar(
  geometry_msgs::msg::TransformStamped::ConstSharedPtr & TF_msg_ptr)
{
  TF_baselink2lidar_ptr_ = TF_msg_ptr;
}

void NDTScanMatcher::callbackSensorPoints(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  // const std::string & sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  const rclcpp::Time sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

  boost::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
  // get TF base to sensor
  // auto TF_base_to_sensor_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  // getTransform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);
  // const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);

  if (!TF_baselink2lidar_ptr_) {
    std::cout << "transform from baselink to lidar is empty, waiting for this message..." << std::endl;
    return;
  }
  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_baselink2lidar_ptr_);
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(
    *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);
  ndt_ptr_->setInputSource(sensor_points_baselinkTF_ptr);

  // start of critical section for initial_pose_msg_ptr_array_
  std::unique_lock<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);
  // check
  if (initial_pose_msg_ptr_array_.size() <= 1) {
    std::cerr << "No Pose" << std::endl;
    return;
  }
  // searchNNPose using timestamp
  auto initial_pose_old_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  auto initial_pose_new_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getNearestTimeStampPose(
    initial_pose_msg_ptr_array_, sensor_ros_time, initial_pose_old_msg_ptr,
    initial_pose_new_msg_ptr);
  popOldPose(initial_pose_msg_ptr_array_, sensor_ros_time);

  // check the time stamp
  bool valid_old_timestamp = validateTimeStampDifference(
    initial_pose_old_msg_ptr->header.stamp, sensor_ros_time, initial_pose_timeout_sec_);
  bool valid_new_timestamp = validateTimeStampDifference(
    initial_pose_new_msg_ptr->header.stamp, sensor_ros_time, initial_pose_timeout_sec_);

  // check the position jumping (ex. immediately after the initial pose estimation)
  bool valid_new_to_old_distance = validatePositionDifference(
    initial_pose_old_msg_ptr->pose.pose.position, initial_pose_new_msg_ptr->pose.pose.position,
    initial_pose_distance_tolerance_m_);

  // must all validations are true
  if (!(valid_old_timestamp && valid_new_timestamp && valid_new_to_old_distance)) {
    std::cerr << "Validation error." << std::endl;
    return;
  }

  // If regularization is enabled and available, set pose to NDT for regularization
  if (regularization_enabled_ && (ndt_implement_type_ == NDTImplementType::OMP)) {
    ndt_ptr_->unsetRegularizationPose();
    std::optional<Eigen::Matrix4f> pose_opt = interpolateRegularizationPose(sensor_ros_time);
    if (pose_opt.has_value()) {
      ndt_ptr_->setRegularizationPose(pose_opt.value());
      std::cout << "Regularization pose is set to NDT" << std::endl;
    }
  }

  const auto initial_pose_msg =
    interpolatePose(*initial_pose_old_msg_ptr, *initial_pose_new_msg_ptr, sensor_ros_time);

  // enf of critical section for initial_pose_msg_ptr_array_
  initial_pose_array_lock.unlock();

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_with_cov_msg_;
  initial_pose_with_cov_msg_.header = initial_pose_msg.header;
  initial_pose_with_cov_msg_.pose.pose = initial_pose_msg.pose;

  if (ndt_ptr_->getInputTarget() == nullptr) {
    std::cerr << "No MAP!" << std::endl;
    return;
  }
  // align
  const Eigen::Affine3d initial_pose_affine = fromRosPoseToEigen(initial_pose_with_cov_msg_.pose.pose);
  const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  key_value_stdmap_["state"] = "Aligning";
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);
  key_value_stdmap_["state"] = "Sleeping";

  const Eigen::Matrix4f result_pose_matrix = ndt_ptr_->getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::msg::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    result_pose_matrix_array = ndt_ptr_->getFinalTransformationArray();
  std::vector<geometry_msgs::msg::Pose> result_pose_msg_array;
  for (const auto & pose_matrix : result_pose_matrix_array) {
    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose_matrix.cast<double>();
    const geometry_msgs::msg::Pose pose_msg = tf2::toMsg(pose_affine);
    result_pose_msg_array.push_back(pose_msg);
  }

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

  const float transform_probability = ndt_ptr_->getTransformationProbability();
  const float nearest_voxel_transformation_likelihood =
    ndt_ptr_->getNearestVoxelTransformationLikelihood();
  const int iteration_num = ndt_ptr_->getFinalNumIteration();

  bool is_ok_iteration_num = iteration_num < ndt_ptr_->getMaximumIterations() + 2;
  if (!is_ok_iteration_num) {
    std::cout << "The number of iterations has reached its upper limit. "
      "The number of iterations: " << iteration_num << " Limit: " << ndt_ptr_->getMaximumIterations() + 2 << std::endl;
  }

  bool is_local_optimal_solution_oscillation = false;
  if (!is_ok_iteration_num) {
    is_local_optimal_solution_oscillation = isLocalOptimalSolutionOscillation(
      result_pose_matrix_array, oscillation_threshold_, inversion_vector_threshold_);
  }

  bool is_ok_converged_param = false;
  if (converged_param_type_ == ConvergedParamType::TRANSFORM_PROBABILITY) {
    is_ok_converged_param = transform_probability > converged_param_transform_probability_;
    if (!is_ok_converged_param) {
      std::cout << "Transform Probability is below the threshold. "
        "Score: " << transform_probability <<  " Threshold:" << converged_param_transform_probability_ << std::endl;
    }
  } else if (converged_param_type_ == ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    is_ok_converged_param = nearest_voxel_transformation_likelihood >
                            converged_param_nearest_voxel_transformation_likelihood_;
    if (!is_ok_converged_param) {
      std::cout << "Nearest Voxel Transform Probability is below the threshold. "
        "Score: " << nearest_voxel_transformation_likelihood <<  " Threshold:" << converged_param_nearest_voxel_transformation_likelihood_ << std::endl;
    }
  } else {
    is_ok_converged_param = false;
    std::cerr << "Unknown converged param type." << std::endl;
  }

  is_converged = false;
  static size_t skipping_publish_num = 0;
  if (is_ok_iteration_num && is_ok_converged_param) {
    is_converged = true;
    skipping_publish_num = 0;
  } else {
    is_converged = false;
    ++skipping_publish_num;
    std::cout << "Not Converged" << std::endl;
  }

  // publish setting
  ndt_pose_msg_.header.stamp = sensor_ros_time;
  ndt_pose_msg_.header.frame_id = map_frame_;
  ndt_pose_msg_.pose = result_pose_msg;

  ndt_pose_with_cov_msg_.header.stamp = sensor_ros_time;
  ndt_pose_with_cov_msg_.header.frame_id = map_frame_;
  ndt_pose_with_cov_msg_.pose.pose = result_pose_msg;
  ndt_pose_with_cov_msg_.pose.covariance = output_pose_covariance_;

  // tf (map frame to ndt_base_frame frame)
  // it seems this tf does not need to be published
  setTF(ndt_base_frame_, ndt_pose_msg_);

  // aligned point cloud
  auto sensor_points_mapTF_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
  pcl::transformPointCloud(
    *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_aligned_pose_msg_);
  sensor_aligned_pose_msg_.header.stamp = sensor_ros_time;
  sensor_aligned_pose_msg_.header.frame_id = map_frame_;

  // the debug msg construction
  visualization_msgs::msg::MarkerArray ndt_marker_msg_;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1);
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::msg::Marker::ADD;
  for (const auto & pose_msg : result_pose_msg_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = ExchangeColorCrc((1.0 * i) / 15.0);
    ndt_marker_msg_.markers.push_back(marker);
  }
  // TODO(Tier IV): delete old marker
  for (; i < ndt_ptr_->getMaximumIterations() + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = ExchangeColorCrc(0);
    ndt_marker_msg_.markers.push_back(marker);
  }

  exe_time_msg_ = makeFloat32Stamped(sensor_ros_time, exe_time);
  transform_probability_msg_ = makeFloat32Stamped(sensor_ros_time, transform_probability);
  nearest_voxel_transformation_likelihood_msg_ = 
    makeFloat32Stamped(sensor_ros_time, nearest_voxel_transformation_likelihood);
  iteration_num_msg_ = makeInt32Stamped(sensor_ros_time, iteration_num);

  const float initial_to_result_distance =
    norm(initial_pose_with_cov_msg_.pose.pose.position, ndt_pose_with_cov_msg_.pose.pose.position);
  initial_to_result_distance_msg_ = 
    makeFloat32Stamped(sensor_ros_time, initial_to_result_distance);
  const float initial_to_result_distance_old =
    norm(initial_pose_old_msg_ptr->pose.pose.position, ndt_pose_with_cov_msg_.pose.pose.position);
  initial_to_result_distance_old_msg_ =
    makeFloat32Stamped(sensor_ros_time, initial_to_result_distance_old);
  const float initial_to_result_distance_new =
    norm(initial_pose_new_msg_ptr->pose.pose.position, ndt_pose_with_cov_msg_.pose.pose.position);
  initial_to_result_distance_new_msg_ = 
    makeFloat32Stamped(sensor_ros_time, initial_to_result_distance_new);

  key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap_["nearest_voxel_transformation_likelihood"] =
    std::to_string(nearest_voxel_transformation_likelihood);
  key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);
  if (is_local_optimal_solution_oscillation) {
    key_value_stdmap_["is_local_optimal_solution_oscillation"] = "1";
  } else {
    key_value_stdmap_["is_local_optimal_solution_oscillation"] = "0";
  }
}

geometry_msgs::msg::PoseWithCovarianceStamped NDTScanMatcher::alignUsingMonteCarlo(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    std::cout << "No Map or Sensor PointCloud" << std::endl;
    return geometry_msgs::msg::PoseWithCovarianceStamped();
  }

  // generateParticle
  const auto initial_poses =
    createRandomPoseArray(initial_pose_with_cov, initial_estimate_particles_num_);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  for (unsigned int i = 0; i < initial_poses.size(); i++) {
    const auto & initial_pose = initial_poses[i];

    const Eigen::Affine3d initial_pose_affine = fromRosPoseToEigen(initial_pose);
    const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

    ndt_ptr->align(*output_cloud, initial_pose_matrix);

    const Eigen::Matrix4f result_pose_matrix = ndt_ptr->getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::msg::Pose result_pose = tf2::toMsg(result_pose_affine);

    const auto transform_probability = ndt_ptr->getTransformationProbability();
    const auto num_iteration = ndt_ptr->getFinalNumIteration();

    Particle particle(initial_pose, result_pose, transform_probability, num_iteration);
    particle_array.push_back(particle);
    ndt_monte_carlo_initial_pose_marker_msg_ = makeDebugMarkers(
      time_utils::to_message(std::chrono::system_clock::now()), map_frame_, 
      tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1), particle, i);

    auto sensor_points_mapTF_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    const auto sensor_points_baselinkTF_ptr = ndt_ptr->getInputSource();
    pcl::transformPointCloud(
      *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
    
    sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
    sensor_points_mapTF_msg.header.stamp = initial_pose_with_cov.header.stamp;
    sensor_points_mapTF_msg.header.frame_id = map_frame_;
    sensor_aligned_pose_msg_ = sensor_points_mapTF_msg;
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = initial_pose_with_cov.header.stamp;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;
  // ndt_pose_with_covariance_msg_ = result_pose_with_cov_msg;

  return result_pose_with_cov_msg;
}

void NDTScanMatcher::setTF(
  const std::string & child_frame_id, const geometry_msgs::msg::PoseStamped & pose_msg)
{
  TF_map2ndtbase_msg_ = tier4_autoware_utils::pose2transform(pose_msg, child_frame_id);
}

bool NDTScanMatcher::validateTimeStampDifference(
  const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
  const double time_tolerance_sec)
{
  const double dt = std::abs((target_time - reference_time).seconds());
  if (dt > time_tolerance_sec) {
    std::cout << "Validation error. The reference time is "<< reference_time.seconds() <<  
    "[sec], but the target time is "<< target_time.seconds() << 
    "[sec] the difference is "<< dt << "[sec] (the tolerance is" << time_tolerance_sec << "[sec])." << std::endl;
    return false;
  }
  return true;
}

bool NDTScanMatcher::validatePositionDifference(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Point & reference_point,
  const double distance_tolerance_m_)
{
  double distance = norm(target_point, reference_point);
  if (distance > distance_tolerance_m_) {
    std::cout << "Validation error. The distance from reference position to target position is "<< 
      distance << "[m] (the tolerance is " << distance_tolerance_m_ << "[m])."<< std::endl;
    return false;
  }
  return true;
}

std::optional<Eigen::Matrix4f> NDTScanMatcher::interpolateRegularizationPose(
  const rclcpp::Time & sensor_ros_time)
{
  if (regularization_pose_msg_ptr_array_.empty()) {
    return std::nullopt;
  }

  // synchronization
  auto regularization_old_msg_ptr =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  auto regularization_new_msg_ptr =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getNearestTimeStampPose(
    regularization_pose_msg_ptr_array_, sensor_ros_time, regularization_old_msg_ptr,
    regularization_new_msg_ptr);
  popOldPose(regularization_pose_msg_ptr_array_, sensor_ros_time);

  const geometry_msgs::msg::PoseStamped regularization_pose_msg =
    interpolatePose(*regularization_old_msg_ptr, *regularization_new_msg_ptr, sensor_ros_time);
  // if the interpolatePose fails, 0.0 is stored in the stamp
  if (rclcpp::Time(regularization_pose_msg.header.stamp).seconds() == 0.0) {
    return std::nullopt;
  }

  Eigen::Affine3d regularization_pose_affine;
  tf2::fromMsg(regularization_pose_msg.pose, regularization_pose_affine);
  return regularization_pose_affine.matrix().cast<float>();
}


std::unique_ptr<NDTScanMatcher> new_operator(const NDTScanMatcherConfig &cfg)
{
  return std::make_unique<NDTScanMatcher>(cfg);
}

OnInputResult on_input(NDTScanMatcher &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
  // input process.  
  std::cout << "NDTScanMatcher operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;
  
  // deseralize
  std::stringstream ss; // any(in/out) stream can be used
  copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));

  if (id == "initial_pose") {
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss); // Create an input archive
      iarchive(initial_pose_ptr); // Read the data from the archive
    }
    op.callbackInitialPose(initial_pose_ptr);
  } else if (id == "tf_baselink2lidar") {
    geometry_msgs::msg::TransformStamped::ConstSharedPtr TF_baselink2lidar_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(TF_baselink2lidar_ptr);
    }
    op.callback_tf_baselink2lidar(TF_baselink2lidar_ptr);
  } else if (id == "regularization_pose") {
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr regularization_pose_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(regularization_pose_ptr);
    }
    op.callbackRegularizationPose(regularization_pose_ptr);
  }
  
  if (id == "map_points") {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(map_points_ptr);
    }
    op.callbackMapPoints(map_points_ptr);
  } else if (id == "align_request_pose") {
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr align_request_pose_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(align_request_pose_ptr);
    }
    op.serviceNDTAlign(align_request_pose_ptr);
    // output construct
    ss.clear();
    {
      cereal::PortableBinaryOutputArchive oarchive(ss); // Create an output archive
      oarchive(op.get_aligned_response_msg_ptr()); // Write the data to the archive
    }
    if (op.get_is_align_succeeded()){
      std::string str = ss.str();
      // string --> unsigned char --> rust::Slice 
      auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
      rust::Slice<const uint8_t> out_slice_pose{uchar.data(), uchar.size()};
      auto send_result_pose = send_output(output_sender, rust::Str("aligned_response_pose"), out_slice_pose);
      OnInputResult result_pose = {send_result_pose.error, false};
      return result_pose;
    }
  } else if (id == "downsampled_pointcloud") {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(pointcloud_ptr);
    }
    op.callbackSensorPoints(pointcloud_ptr);

    // output construct
    ss.clear();
    {
      cereal::PortableBinaryOutputArchive oarchive(ss); // Create an output archive
      oarchive(op.get_ndt_pose_msg_ptr()); // Write the data to the archive
    }
    if (op.get_is_converged()){
      std::string str = ss.str();
      auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
      rust::Slice<const uint8_t> out_slice_pose{uchar.data(), uchar.size()};
      auto send_result_pose = send_output(output_sender, rust::Str("ndt_pose_with_cov"), out_slice_pose);
      OnInputResult result_pose = {send_result_pose.error, false};
      return result_pose;
    }
  }
  // Set the default return
  return OnInputResult{rust::String(""), false};
}
