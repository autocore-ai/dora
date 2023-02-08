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

#include "ekf_localizer/ekf_localizer.hpp"
#include "ekf_localizer/ffi.rs.h"

#include "ekf_localizer/mahalanobis.hpp"
#include "ekf_localizer/matrix_types.hpp"
#include "ekf_localizer/measurement.hpp"
#include "ekf_localizer/numeric.hpp"
#include "ekf_localizer/state_index.hpp"
#include "ekf_localizer/state_transition.hpp"

#include <time_utils/time_utils.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <stdio.h>  // for using printf

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_geometry_msgs.hpp"
#include "cereal_ros_msgs/cereal_nav_msgs.hpp"
#include "cereal_ros_msgs/cereal_tier4_debug_msgs.hpp"

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
// #define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_INFO(...) {if (show_debug_info_) {printf(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on

EKFLocalizer::EKFLocalizer(const EKFLocalizerConfig &cfg)
{
  show_debug_info_ = cfg.show_debug_info;
  ekf_rate_ = cfg.predict_frequency;
  ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);
  tf_rate_ = cfg.tf_rate;
  enable_yaw_bias_estimation_ = cfg.enable_yaw_bias_estimation;
  extend_state_step_ = cfg.extend_state_step;
  pose_frame_id_ = static_cast<std::string>(cfg.pose_frame_id);

  /* pose measurement */
  pose_additional_delay_ = cfg.pose_additional_delay;
  pose_measure_uncertainty_time_ = cfg.pose_measure_uncertainty_time;
  pose_gate_dist_ = cfg.pose_gate_dist;  // Mahalanobis limit
  pose_smoothing_steps_ = cfg.pose_smoothing_steps;

  /* twist measurement */
  twist_additional_delay_ = cfg.twist_additional_delay;
  twist_gate_dist_ = cfg.twist_gate_dist;  // Mahalanobis limit
  twist_smoothing_steps_ = cfg.twist_smoothing_steps;

  /* process noise */
  proc_stddev_yaw_c_ = cfg.proc_stddev_yaw_c;
  proc_stddev_vx_c_ = cfg.proc_stddev_vx_c;
  proc_stddev_wz_c_ = cfg.proc_stddev_wz_c;

  /* convert to continuous to discrete */
  proc_cov_vx_d_ = std::pow(proc_stddev_vx_c_ * ekf_dt_, 2.0);
  proc_cov_wz_d_ = std::pow(proc_stddev_wz_c_ * ekf_dt_, 2.0);
  proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c_ * ekf_dt_, 2.0);

  is_initialized_ = false;

  dim_x_ = 6;   // x, y, yaw, yaw_bias, vx, wz
  dim_x_ex_ = dim_x_ * extend_state_step_;

  initEKF();

  z_filter_.set_proc_dev(1.0);
  roll_filter_.set_proc_dev(0.01);
  pitch_filter_.set_proc_dev(0.01);
}

/*
 * updatePredictFrequency
 */
void EKFLocalizer::updatePredictFrequency()
{
  if (last_predict_time_) {
    if (std::chrono::system_clock::now() < *last_predict_time_) {
      std::cout << "Detected jump back in time" << std::endl;
    } else {
      const double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now() - *last_predict_time_).count() / 1000.0;
      ekf_rate_ = 1.0 / elapsed;
      DEBUG_INFO("[EKF] update ekf_rate_ to %f hz", ekf_rate_);
      ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);

      /* Update discrete proc_cov*/
      proc_cov_vx_d_ = std::pow(proc_stddev_vx_c_ * ekf_dt_, 2.0);
      proc_cov_wz_d_ = std::pow(proc_stddev_wz_c_ * ekf_dt_, 2.0);
      proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c_ * ekf_dt_, 2.0);
    }
  }
  last_predict_time_ = std::make_shared<const std::chrono::system_clock::time_point>(
    std::chrono::system_clock::now());
}

/*
 * timerCallback
 */
void EKFLocalizer::timerCallback()
{
  is_ekf_result_set_ = false;
  if (!is_initialized_) {
    return;
  }

  DEBUG_INFO("========================= timer called =========================");

  /* update predict frequency with measured timer rate */
  updatePredictFrequency();

  /* predict model in EKF */
  stop_watch_.tic();
  DEBUG_INFO("------------------------- start prediction -------------------------");
  predictKinematicsModel();
  DEBUG_INFO("[EKF] predictKinematicsModel calc time = %f [ms]", stop_watch_.toc());
  DEBUG_INFO("------------------------- end prediction -------------------------\n");

  /* pose measurement update */
  if (!current_pose_info_queue_.empty()) {
    DEBUG_INFO("------------------------- start Pose -------------------------");
    stop_watch_.tic();

    int pose_info_queue_size = static_cast<int>(current_pose_info_queue_.size());
    for (int i = 0; i < pose_info_queue_size; ++i) {
      PoseInfo pose_info = current_pose_info_queue_.front();
      current_pose_info_queue_.pop();
      measurementUpdatePose(*pose_info.pose);
      ++pose_info.counter;
      if (pose_info.counter < pose_info.smoothing_steps) {
        current_pose_info_queue_.push(pose_info);
      }
    }
    DEBUG_INFO("[EKF] measurementUpdatePose calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO("------------------------- end Pose -------------------------\n");
  }

  /* twist measurement update */
  if (!current_twist_info_queue_.empty()) {
    DEBUG_INFO("------------------------- start Twist -------------------------");
    stop_watch_.tic();

    int twist_info_queue_size = static_cast<int>(current_twist_info_queue_.size());
    for (int i = 0; i < twist_info_queue_size; ++i) {
      TwistInfo twist_info = current_twist_info_queue_.front();
      current_twist_info_queue_.pop();
      measurementUpdateTwist(*twist_info.twist);
      ++twist_info.counter;
      if (twist_info.counter < twist_info.smoothing_steps) {
        current_twist_info_queue_.push(twist_info);
      }
    }
    DEBUG_INFO("[EKF] measurementUpdateTwist calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO("------------------------- end Twist -------------------------\n");
  }

  const double x = ekf_.getXelement(IDX::X);
  const double y = ekf_.getXelement(IDX::Y);
  const double z = z_filter_.get_x();

  const double biased_yaw = ekf_.getXelement(IDX::YAW);
  const double yaw_bias = ekf_.getXelement(IDX::YAWB);

  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();
  const double yaw = biased_yaw + yaw_bias;
  const double vx = ekf_.getXelement(IDX::VX);
  const double wz = ekf_.getXelement(IDX::WZ);

  current_ekf_pose_.header.frame_id = pose_frame_id_;
  current_ekf_pose_.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  current_ekf_pose_.pose.position.x = x;
  current_ekf_pose_.pose.position.y = y;
  current_ekf_pose_.pose.position.z = z;
  current_ekf_pose_.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, yaw);

  current_biased_ekf_pose_ = current_ekf_pose_;
  current_biased_ekf_pose_.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, biased_yaw);

  current_ekf_twist_.header.frame_id = "base_link";
  current_ekf_twist_.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  current_ekf_twist_.twist.linear.x = vx;
  current_ekf_twist_.twist.angular.z = wz;

  // for debug
  // printf("the ekf output: position-xyz(%f, %f, %f), orientation-rpy(%f, %f, %f), linear-x(%f), angular-z(%f)\n", x, y, z, roll, pitch, yaw, vx, wz);

  /* set ekf result */
  setEstimateResult();
  is_ekf_result_set_ = true;
}

void EKFLocalizer::showCurrentX()
{
  if (show_debug_info_) {
    Eigen::MatrixXd X(dim_x_, 1);
    ekf_.getLatestX(X);
    DEBUG_PRINT_MAT(X.transpose());
  }
}

/*
 * timerTFCallback
 */
void EKFLocalizer::timerTFCallback()
{
  if (!is_initialized_) {
    return;
  }

  if (current_ekf_pose_.header.frame_id == "") {
    return;
  }

  TF_map2baselink_msg_.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  TF_map2baselink_msg_.header.frame_id = current_ekf_pose_.header.frame_id;
  TF_map2baselink_msg_.child_frame_id = "base_link";
  TF_map2baselink_msg_.transform.translation.x = current_ekf_pose_.pose.position.x;
  TF_map2baselink_msg_.transform.translation.y = current_ekf_pose_.pose.position.y;
  TF_map2baselink_msg_.transform.translation.z = current_ekf_pose_.pose.position.z;

  TF_map2baselink_msg_.transform.rotation.x = current_ekf_pose_.pose.orientation.x;
  TF_map2baselink_msg_.transform.rotation.y = current_ekf_pose_.pose.orientation.y;
  TF_map2baselink_msg_.transform.rotation.z = current_ekf_pose_.pose.orientation.z;
  TF_map2baselink_msg_.transform.rotation.w = current_ekf_pose_.pose.orientation.w;
}

/*
 * callbackInitialPose
 */
void EKFLocalizer::callbackInitialPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
  if (initialpose->header.frame_id != pose_frame_id_) {
    std::cerr << "initial pose frame don't match with the required pose frame: " << pose_frame_id_ << std::endl;
    // DEBUG_INFO("initial pose frame don't match with the required pose frame: %s", pose_frame_id_.c_str());
  }

  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  // TODO(mitsudome-r) need mutex

  X(IDX::X) = initialpose->pose.pose.position.x;
  X(IDX::Y) = initialpose->pose.pose.position.y;
  current_ekf_pose_.pose.position.z = initialpose->pose.pose.position.z;
  X(IDX::YAW) = tf2::getYaw(initialpose->pose.pose.orientation);
  X(IDX::YAWB) = 0.0;
  X(IDX::VX) = 0.0;
  X(IDX::WZ) = 0.0;

  P(IDX::X, IDX::X) = initialpose->pose.covariance[6 * 0 + 0];
  P(IDX::Y, IDX::Y) = initialpose->pose.covariance[6 * 1 + 1];
  P(IDX::YAW, IDX::YAW) = initialpose->pose.covariance[6 * 5 + 5];
  if (enable_yaw_bias_estimation_) {
    P(IDX::YAWB, IDX::YAWB) = 0.0001;
  }
  P(IDX::VX, IDX::VX) = 0.01;
  P(IDX::WZ, IDX::WZ) = 0.01;

  ekf_.init(X, P, extend_state_step_);

  updateSimple1DFilters(*initialpose);

  while (!current_pose_info_queue_.empty()) current_pose_info_queue_.pop();

  is_initialized_ = true;
}

/*
 * callbackPoseWithCovariance
 */
void EKFLocalizer::callbackPoseWithCovariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  PoseInfo pose_info = {msg, 0, pose_smoothing_steps_};
  current_pose_info_queue_.push(pose_info);

  updateSimple1DFilters(*msg);
}

/*
 * callbackTwistWithCovariance
 */
void EKFLocalizer::callbackTwistWithCovariance(
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  TwistInfo twist_info = {msg, 0, twist_smoothing_steps_};
  current_twist_info_queue_.push(twist_info);
}

/*
 * initEKF
 */
void EKFLocalizer::initEKF()
{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15;  // for x & y
  P(IDX::YAW, IDX::YAW) = 50.0;                                            // for yaw
  if (enable_yaw_bias_estimation_) {
    P(IDX::YAWB, IDX::YAWB) = 50.0;  // for yaw bias
  }
  P(IDX::VX, IDX::VX) = 1000.0;  // for vx
  P(IDX::WZ, IDX::WZ) = 50.0;    // for wz

  ekf_.init(X, P, extend_state_step_);
}

/*
 * predictKinematicsModel
 */
void EKFLocalizer::predictKinematicsModel()
{
  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k + b_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k + b_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   * b_{k+1}   = b_k
   * vx_{k+1}  = vz_k
   * wz_{k+1}  = wz_k
   *
   * (b_k : yaw_bias_k)
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, -vx*sin(yaw+b)*dt, -vx*sin(yaw+b)*dt, cos(yaw+b)*dt,  0]
   *     [ 0, 1,  vx*cos(yaw+b)*dt,  vx*cos(yaw+b)*dt, sin(yaw+b)*dt,  0]
   *     [ 0, 0,                 1,                 0,             0, dt]
   *     [ 0, 0,                 0,                 1,             0,  0]
   *     [ 0, 0,                 0,                 0,             1,  0]
   *     [ 0, 0,                 0,                 0,             0,  1]
   */

  Eigen::MatrixXd X_curr(dim_x_, 1);  // current state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  Eigen::MatrixXd P_curr;
  ekf_.getLatestP(P_curr);

  const double dt = ekf_dt_;

  const Vector6d X_next = predictNextState(X_curr, dt);
  const Matrix6d A = createStateTransitionMatrix(X_curr, dt);
  const Matrix6d Q = processNoiseCovariance(proc_cov_yaw_d_, proc_cov_vx_d_, proc_cov_wz_d_);

  ekf_.predictWithDelay(X_next, A, Q);

  // debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdatePose
 */
void EKFLocalizer::measurementUpdatePose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  if (pose.header.frame_id != pose_frame_id_) {
    // std::cerr << "required pose frame_id is: " << pose_frame_id_ << 
    //   " but input pose frame is: " << pose.header.frame_id << ". They must be same." << std::endl;
    DEBUG_INFO("pose frame_id is %s, but the required pose_frame is %s. They must be same.",
        pose.header.frame_id.c_str(), pose_frame_id_.c_str());
  }
  Eigen::MatrixXd X_curr(dim_x_, 1);  // current state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output
  const auto t_curr = std::chrono::system_clock::now();
  /* Calculate delay step */
  double delay_time = std::chrono::duration_cast<std::chrono::milliseconds>(
    t_curr - time_utils::from_message(pose.header.stamp)).count()/1000.0 + pose_additional_delay_;
  
  if (delay_time < 0.0) {
    // std::cerr << "Pose time stamp is inappropriate (delay = " << delay_time << "[s]), set delay to 0[s]." << std::endl;
    DEBUG_INFO("Pose time stamp is inappropriate (delay = %f [s]), set delay to 0[s].", delay_time);
    delay_time = 0.0;
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1) {
    // std::cerr << "Pose delay exceeds the compensation limit, ignored. delay:" << delay_time << 
    //   "[s], limit = extend_state_step * ekf_dt : " << extend_state_step_ * ekf_dt_ << "[s]" << std::endl;
    DEBUG_INFO("Pose delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
      "extend_state_step * ekf_dt : %f [s]", delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  DEBUG_INFO("delay_time: %f [s]", delay_time);

  /* Set yaw */
  double yaw = tf2::getYaw(pose.pose.pose.orientation);
  const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error = normalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << pose.pose.pose.position.x, pose.pose.pose.position.y, yaw;

  if (hasNan(y) || hasInf(y)) {
    std::cout << "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message." << std::endl;
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::X),
    ekf_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;
  Eigen::MatrixXd P_curr, P_y;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(0, 0, dim_y, dim_y);
  if (!mahalanobisGate(pose_gate_dist_, y_ekf, y, P_y)) {
    std::cerr << "[EKF] Pose measurement update, mahalanobis distance is over limit. ignore measurement data." << std::endl;
    return;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  const Eigen::Matrix<double, 3, 6> C = poseMeasurementMatrix();
  const Eigen::Matrix3d R = poseMeasurementCovariance(pose.pose.covariance, pose_smoothing_steps_);

  ekf_.updateWithDelay(y, C, R, delay_step);

  // debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdateTwist
 */
void EKFLocalizer::measurementUpdateTwist(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist)
{
  if (twist.header.frame_id != "base_link") {
    std::cout << "twist frame_id must be base_link" << std::endl;
  }

  Eigen::MatrixXd X_curr(dim_x_, 1);  // current state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 2;  // vx, wz
  const auto t_curr = std::chrono::system_clock::now();
  /* Calculate delay step */
  double delay_time = std::chrono::duration_cast<std::chrono::milliseconds>(
    t_curr - time_utils::from_message(twist.header.stamp)).count()/1000.0 + twist_additional_delay_;
  
  if (delay_time < 0.0) {
    // std::cerr << "Twist time stamp is inappropriate (delay = " << delay_time << " [s]), set delay to 0[s]." << std::endl;
    DEBUG_INFO("Twist time stamp is inappropriate (delay = %f [s]), set delay to 0[s].", delay_time);
    delay_time = 0.0;
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1) {
    // std::cerr << "Twist delay exceeds the compensation limit, ignored. delay: " << delay_time << 
    //   "[s], limit = extend_state_step * ekf_dt : " << extend_state_step_ * ekf_dt_ << "[s]" << std::endl;
    DEBUG_INFO("Twist delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
      "extend_state_step * ekf_dt : %f [s]", delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  DEBUG_INFO("delay_time: %f [s]", delay_time);

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << twist.twist.twist.linear.x, twist.twist.twist.angular.z;

  if (hasNan(y) || hasInf(y)) {
    std::cout << "[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message." << std::endl;
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::VX),
    ekf_.getXelement(delay_step * dim_x_ + IDX::WZ);
  Eigen::MatrixXd P_curr, P_y;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(4, 4, dim_y, dim_y);
  if (!mahalanobisGate(twist_gate_dist_, y_ekf, y, P_y)) {
    std::cerr << "[EKF] Twist measurement update, mahalanobis distance is over limit. ignore measurement data." << std::endl;
    return;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  const Eigen::Matrix<double, 2, 6> C = twistMeasurementMatrix();
  const Eigen::Matrix2d R =
    twistMeasurementCovariance(twist.twist.covariance, twist_smoothing_steps_);

  ekf_.updateWithDelay(y, C, R, delay_step);

  // debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * publishEstimateResult
 */
void EKFLocalizer::setEstimateResult()
{
  builtin_interfaces::msg::Time current_time = 
    time_utils::to_message(std::chrono::system_clock::now());
  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P(dim_x_, dim_x_);
  ekf_.getLatestX(X);
  ekf_.getLatestP(P);

  /* publish latest pose */
  // pose_msg_ = current_ekf_pose_;
  // biased_pose_msg_ = current_biased_ekf_pose_;

  /* publish latest pose with covariance */
  pose_with_cov_msg_.header.stamp = current_time;
  pose_with_cov_msg_.header.frame_id = current_ekf_pose_.header.frame_id;
  pose_with_cov_msg_.pose.pose = current_ekf_pose_.pose;
  pose_with_cov_msg_.pose.covariance[0] = P(IDX::X, IDX::X);
  pose_with_cov_msg_.pose.covariance[1] = P(IDX::X, IDX::Y);
  pose_with_cov_msg_.pose.covariance[5] = P(IDX::X, IDX::YAW);
  pose_with_cov_msg_.pose.covariance[6] = P(IDX::Y, IDX::X);
  pose_with_cov_msg_.pose.covariance[7] = P(IDX::Y, IDX::Y);
  pose_with_cov_msg_.pose.covariance[11] = P(IDX::Y, IDX::YAW);
  pose_with_cov_msg_.pose.covariance[30] = P(IDX::YAW, IDX::X);
  pose_with_cov_msg_.pose.covariance[31] = P(IDX::YAW, IDX::Y);
  pose_with_cov_msg_.pose.covariance[35] = P(IDX::YAW, IDX::YAW);

  biased_pose_with_cov_msg_ = pose_with_cov_msg_;
  biased_pose_with_cov_msg_.pose.pose = current_biased_ekf_pose_.pose;

  /* publish latest twist */
  // twist_msg_ = current_ekf_twist_;

  /* publish latest twist with covariance */
  twist_with_cov_msg_.header.stamp = current_time;
  twist_with_cov_msg_.header.frame_id = current_ekf_twist_.header.frame_id;
  twist_with_cov_msg_.twist.twist = current_ekf_twist_.twist;
  twist_with_cov_msg_.twist.covariance[0] = P(IDX::VX, IDX::VX);
  twist_with_cov_msg_.twist.covariance[5] = P(IDX::VX, IDX::WZ);
  twist_with_cov_msg_.twist.covariance[30] = P(IDX::WZ, IDX::VX);
  twist_with_cov_msg_.twist.covariance[35] = P(IDX::WZ, IDX::WZ);

  /* publish yaw bias */
  yaw_bias_msg_.stamp = current_time;
  yaw_bias_msg_.data = X(IDX::YAWB);

  /* publish latest odometry */
  kinematic_msg_.header.stamp = current_time;
  kinematic_msg_.header.frame_id = current_ekf_pose_.header.frame_id;
  kinematic_msg_.child_frame_id = "base_link";
  kinematic_msg_.pose = pose_with_cov_msg_.pose;
  kinematic_msg_.twist = twist_with_cov_msg_.twist;

  /* debug measured pose */
  if (!current_pose_info_queue_.empty()) {
    measured_pose_msg_.pose = current_pose_info_queue_.back().pose->pose.pose;
    measured_pose_msg_.header.stamp = current_time;
  }

  /* debug publish */
  double pose_yaw = 0.0;
  if (!current_pose_info_queue_.empty()) {
    pose_yaw = tf2::getYaw(current_pose_info_queue_.back().pose->pose.pose.orientation);
  }

  debug_msg_.stamp = current_time;
  debug_msg_.data.push_back(tier4_autoware_utils::rad2deg(X(IDX::YAW)));   // [0] ekf yaw angle
  debug_msg_.data.push_back(tier4_autoware_utils::rad2deg(pose_yaw));      // [1] measurement yaw angle
  debug_msg_.data.push_back(tier4_autoware_utils::rad2deg(X(IDX::YAWB)));  // [2] yaw bias
}

void EKFLocalizer::updateSimple1DFilters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  double z = pose.pose.pose.position.z;
  double roll = 0.0, pitch = 0.0, yaw_tmp = 0.0;

  tf2::Quaternion q_tf;
  tf2::fromMsg(pose.pose.pose.orientation, q_tf);
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw_tmp);

  double z_dev = pose.pose.covariance[2 * 6 + 2];
  double roll_dev = pose.pose.covariance[3 * 6 + 3];
  double pitch_dev = pose.pose.covariance[4 * 6 + 4];

  z_filter_.update(z, z_dev, time_utils::from_message(pose.header.stamp));
  roll_filter_.update(roll, roll_dev, time_utils::from_message(pose.header.stamp));
  pitch_filter_.update(pitch, pitch_dev, time_utils::from_message(pose.header.stamp));
}

std::unique_ptr<EKFLocalizer> new_operator(const EKFLocalizerConfig &cfg)
{
  return std::make_unique<EKFLocalizer>(cfg);
}

OnInputResult on_input(EKFLocalizer &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
  // input process.  
  // std::cout << "EKFLocalizer operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;
  
  // deseralize
  std::stringstream ss; // any(in/out) stream can be used
  copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));
  
  if (id == "initial_pose"){
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(initial_pose_ptr);
    }
    op.callbackInitialPose(initial_pose_ptr);
  } else if (id == "ndt_pose_with_cov"){
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_cov_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(pose_cov_ptr);
    }
    op.callbackPoseWithCovariance(pose_cov_ptr);
  } else if (id == "twist_with_cov"){
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_cov_ptr;
    {
      cereal::PortableBinaryInputArchive iarchive(ss);
      iarchive(twist_cov_ptr);
    }
    op.callbackTwistWithCovariance(twist_cov_ptr);
  } else {
    // timer trigger: TF and pose share a time trigger
    op.timerCallback();
    op.timerTFCallback();
    if(op.is_ekf_result_set()){
      // pose_with_covariance
      ss.str(""); // clear the buffer of ss
      {
        cereal::PortableBinaryOutputArchive oarchive(ss);
        oarchive(op.get_pose_with_cov_msg_ptr());
      }
      std::string str = ss.str();
      auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
      rust::Slice<const uint8_t> out_slice_pose_cov{uchar.data(), uchar.size()};
      auto send_result_pose_cov = send_output(output_sender, rust::Str("ekf_pose_with_cov"), out_slice_pose_cov);
      OnInputResult result_pose_cov = {send_result_pose_cov.error, false};

      // twist_with_covariance
      ss.str("");
      {
        cereal::PortableBinaryOutputArchive oarchive(ss);
        oarchive(op.get_twist_with_cov_msg_ptr());
      }
      str = ss.str();
      uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
      rust::Slice<const uint8_t> out_slice_twist_cov{uchar.data(), uchar.size()};
      auto send_result_twist_cov = send_output(output_sender, rust::Str("ekf_twist_with_cov"), out_slice_twist_cov);
      OnInputResult result_twist_cov = {send_result_twist_cov.error, false};

      // biased_pose_with_covariance
      ss.str("");
      {
        cereal::PortableBinaryOutputArchive oarchive(ss);
        oarchive(op.get_biased_pose_with_cov_msg_ptr());
      }
      str = ss.str();
      uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
      rust::Slice<const uint8_t> out_slice_biased_pose_cov{uchar.data(), uchar.size()};
      auto send_result_biased_pose_cov = send_output(output_sender, rust::Str("ekf_biased_pose_with_cov"), out_slice_biased_pose_cov);
      OnInputResult result_biased_pose_cov = {send_result_biased_pose_cov.error, false};

      // kinematic state
      ss.str("");
      {
        cereal::PortableBinaryOutputArchive oarchive(ss);
        oarchive(op.get_kinematic_msg_ptr());
      }
      str = ss.str();
      uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
      rust::Slice<const uint8_t> out_slice_kinematic{uchar.data(), uchar.size()};
      auto send_result_kinematic = send_output(output_sender, rust::Str("kinematic_state"), out_slice_kinematic);
      OnInputResult result_kinematic = {send_result_kinematic.error, false};

      // TF map2baselink
      ss.str("");
      {
        cereal::PortableBinaryOutputArchive oarchive(ss);
        oarchive(op.get_TF_map2baselink_msg_ptr());
      }
      str = ss.str();
      uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
      rust::Slice<const uint8_t> out_slice_tf_map2baselink{uchar.data(), uchar.size()};
      auto send_result_tf_map2baselink = send_output(output_sender, rust::Str("tf_map2baselink"), out_slice_tf_map2baselink);
      OnInputResult result_tf_map2baselink = {send_result_tf_map2baselink.error, false};
      return result_tf_map2baselink;
    }
  }
  // Set the default return
  return OnInputResult{rust::String(""), false};
}
