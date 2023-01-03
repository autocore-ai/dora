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

// Copyright 2018-2019 Autoware Foundation
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

#ifndef EKF_LOCALIZER__EKF_LOCALIZER_HPP_
#define EKF_LOCALIZER__EKF_LOCALIZER_HPP_

#include "cxx.h"

#include <kalman_filter/kalman_filter.hpp>
#include <kalman_filter/time_delay_kalman_filter.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

struct PoseInfo
{
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose;
  int counter;
  int smoothing_steps;
};

struct TwistInfo
{
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist;
  int counter;
  int smoothing_steps;
};

class Simple1DFilter
{
public:
  Simple1DFilter()
  {
    initialized_ = false;
    x_ = 0;
    dev_ = 1e9;
    proc_dev_x_c_ = 0.0;
    return;
  };
  void init(const double init_obs, const double obs_dev, const std::chrono::system_clock::time_point time)
  {
    x_ = init_obs;
    dev_ = obs_dev;
    latest_time_ = time;
    initialized_ = true;
    return;
  };
  void update(const double obs, const double obs_dev, const std::chrono::system_clock::time_point time)
  {
    if (!initialized_) {
      init(obs, obs_dev, time);
      return;
    }

    // Prediction step (current stddev_)
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(time - latest_time_).count() / 1000.0;
    double proc_dev_x_d = proc_dev_x_c_ * dt * dt;
    dev_ = dev_ + proc_dev_x_d;

    // Update step
    double kalman_gain = dev_ / (dev_ + obs_dev);
    x_ = x_ + kalman_gain * (obs - x_);
    dev_ = (1 - kalman_gain) * dev_;

    latest_time_ = time;
    return;
  };
  void set_proc_dev(const double proc_dev) { proc_dev_x_c_ = proc_dev; }
  double get_x() { return x_; }

private:
  bool initialized_;
  double x_;
  double dev_;
  double proc_dev_x_c_;
  std::chrono::system_clock::time_point latest_time_;
};

struct EKFLocalizerConfig;

class EKFLocalizer
{
public:
  EKFLocalizer(const EKFLocalizerConfig &cfg);
  ~EKFLocalizer(){}

  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr get_pose_with_cov_msg_ptr(){
    return std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(pose_with_cov_msg_);
  }
  nav_msgs::msg::Odometry::ConstSharedPtr get_kinematic_msg_ptr(){
    return std::make_shared<const nav_msgs::msg::Odometry>(kinematic_msg_);
  }
  geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr get_twist_with_cov_msg_ptr(){
    return std::make_shared<const geometry_msgs::msg::TwistWithCovarianceStamped>(twist_with_cov_msg_);
  }
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr get_biased_pose_with_cov_msg_ptr(){
    return std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(biased_pose_with_cov_msg_);
  }
  geometry_msgs::msg::TransformStamped::ConstSharedPtr get_TF_map2baselink_msg_ptr(){
    return std::make_shared<const geometry_msgs::msg::TransformStamped>(TF_map2baselink_msg_);
  }
  bool is_ekf_result_set(){return is_ekf_result_set_;}

  ///computes update & prediction of EKF for each ekf_dt_[s] time
  void timerCallback();
  ///publish tf for tf_rate [Hz]
  void timerTFCallback();
  /// set poseWithCovariance measurement
  void callbackPoseWithCovariance(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  /// set twistWithCovariance measurement
  void callbackTwistWithCovariance(geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  /// set initial_pose to current EKF pose
  void callbackInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

private:
  // geometry_msgs::msg::PoseStamped pose_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov_msg_;
  nav_msgs::msg::Odometry kinematic_msg_;
  // geometry_msgs::msg::TwistStamped twist_msg_;
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov_msg_;
  tier4_debug_msgs::msg::Float64Stamped yaw_bias_msg_;
  // geometry_msgs::msg::PoseStamped biased_pose_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped biased_pose_with_cov_msg_;

  /* debug */
  tier4_debug_msgs::msg::Float64MultiArrayStamped debug_msg_;
  geometry_msgs::msg::PoseStamped measured_pose_msg_;
  /* transform */
  geometry_msgs::msg::TransformStamped TF_map2baselink_msg_;
  
  //!< @brief last predict time
  std::shared_ptr<const std::chrono::system_clock::time_point> last_predict_time_;

  //!< @brief  extended kalman filter instance.
  TimeDelayKalmanFilter ekf_;
  Simple1DFilter z_filter_;
  Simple1DFilter roll_filter_;
  Simple1DFilter pitch_filter_;

  /* parameters */
  bool show_debug_info_;
  double ekf_rate_;                  //!< @brief  EKF predict rate
  double ekf_dt_;                    //!< @brief  = 1 / ekf_rate_
  double tf_rate_;                   //!< @brief  tf publish rate
  bool enable_yaw_bias_estimation_;  //!< @brief for LiDAR mount error.
                                     //!< if true,publish /estimate_yaw_bias
  std::string pose_frame_id_;

  int dim_x_;              //!< @brief  dimension of EKF state
  int extend_state_step_;  //!< @brief  for time delay compensation
  int dim_x_ex_;  //!< @brief  dimension of extended EKF state (dim_x_ * extended_state_step)

  /* Pose */
  double pose_additional_delay_;          //!< @brief  compensated pose delay time =
                                          //!< (pose.header.stamp - now) + additional_delay [s]
  double pose_measure_uncertainty_time_;  //!< @brief  added for measurement covariance
  double pose_rate_;  //!< @brief  pose rate [s], used for covariance calculation
  //!< @brief  the mahalanobis distance threshold to ignore pose measurement
  double pose_gate_dist_;

  /* twist */
  double twist_additional_delay_;  //!< @brief  compensated delay = (twist.header.stamp - now)
                                   //!< + additional_delay [s]
  double twist_rate_;              //!< @brief  rate [s], used for covariance calculation
  //!< @brief  measurement is ignored if the mahalanobis distance is larger than this value.
  double twist_gate_dist_;

  /* process noise standard deviation */
  double proc_stddev_yaw_c_;       //!< @brief  yaw process noise
  double proc_stddev_yaw_bias_c_;  //!< @brief  yaw bias process noise
  double proc_stddev_vx_c_;        //!< @brief  vx process noise
  double proc_stddev_wz_c_;        //!< @brief  wz process noise

  /* process noise variance for discrete model */
  double proc_cov_yaw_d_;       //!< @brief  discrete yaw process noise
  double proc_cov_yaw_bias_d_;  //!< @brief  discrete yaw bias process noise
  double proc_cov_vx_d_;        //!< @brief  discrete process noise in d_vx=0
  double proc_cov_wz_d_;        //!< @brief  discrete process noise in d_wz=0

  bool is_initialized_;
  bool is_ekf_result_set_;

  /* for model prediction */
  std::queue<TwistInfo> current_twist_info_queue_;    //!< @brief current measured pose
  std::queue<PoseInfo> current_pose_info_queue_;      //!< @brief current measured pose
  geometry_msgs::msg::PoseStamped current_ekf_pose_;  //!< @brief current estimated pose
  geometry_msgs::msg::PoseStamped
    current_biased_ekf_pose_;  //!< @brief current estimated pose without yaw bias correction
  geometry_msgs::msg::TwistStamped current_ekf_twist_;  //!< @brief current estimated twist
  std::array<double, 36ul> current_pose_covariance_;
  std::array<double, 36ul> current_twist_covariance_;

  int pose_smoothing_steps_;
  int twist_smoothing_steps_;

  /**
   * @brief initialization of EKF
   */
  void initEKF();

  /**
   * @brief update predict frequency
   */
  void updatePredictFrequency();

  /**
   * @brief compute EKF prediction
   */
  void predictKinematicsModel();

  /**
   * @brief compute EKF update with pose measurement
   * @param pose measurement value
   */
  void measurementUpdatePose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  /**
   * @brief compute EKF update with pose measurement
   * @param twist measurement value
   */
  void measurementUpdateTwist(const geometry_msgs::msg::TwistWithCovarianceStamped & twist);

  /**
   * @brief publish current EKF estimation result
   */
  void setEstimateResult();

  /**
   * @brief for debug
   */
  void showCurrentX();

  void updateSimple1DFilters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
};

std::unique_ptr<EKFLocalizer> new_operator(const EKFLocalizerConfig &cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(EKFLocalizer &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);

#endif  // EKF_LOCALIZER__EKF_LOCALIZER_HPP_
