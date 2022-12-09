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

#pragma once
#include "cxx.h"

// #include <chrono>
#include <memory>
#include <vector>
#include <mutex>
#include <string>

#include <time_utils/time_utils.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

struct NdtLocalizerConfig;

class NdtLocalizer
{
public:
    NdtLocalizer(const NdtLocalizerConfig &cfg);
    ~NdtLocalizer(){}

    geometry_msgs::msg::PoseStamped::ConstSharedPtr get_ndt_pose_msg_ptr(){
        return std::make_shared<const geometry_msgs::msg::PoseStamped>(ndt_pose_msg_);
    }
    std_msgs::msg::Float32::ConstSharedPtr get_iteration_num_msg_ptr(){
        return std::make_shared<const std_msgs::msg::Float32>(iteration_num_msg_);
    }
    std_msgs::msg::Float32::ConstSharedPtr get_transform_probability_msg_ptr(){
        return std::make_shared<const std_msgs::msg::Float32>(transform_probability_msg_);
    }
    geometry_msgs::msg::TransformStamped::ConstSharedPtr get_tf_map2baselink_ptr(){
        return std::make_shared<const geometry_msgs::msg::TransformStamped>(TF_map2baselink_msg_);
    }

    bool get_is_converged(){return is_converged;}
    void callback_init_pose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_conv_msg_ptr);
    void callback_map_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud2_msg_ptr);
    void callback_pointcloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud2_msg_ptr);
    void callback_tf_baselink2lidar(geometry_msgs::msg::TransformStamped::ConstSharedPtr & TF_msg_ptr);
private:
    sensor_msgs::msg::PointCloud2 sensor_aligned_pose_msg_;
    geometry_msgs::msg::PoseStamped ndt_pose_msg_;
    std_msgs::msg::Float32 exe_time_msg_;
    std_msgs::msg::Float32 transform_probability_msg_;
    std_msgs::msg::Float32 iteration_num_msg_;
    
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

    geometry_msgs::msg::TransformStamped::ConstSharedPtr TF_baselink2lidar_ptr_;
    geometry_msgs::msg::TransformStamped TF_map2baselink_msg_;

    Eigen::Matrix4f base_to_sensor_matrix_;
    Eigen::Matrix4f pre_trans, delta_trans;
    bool init_pose = false;
    bool is_converged = true;

    std::string base_frame_;
    std::string map_frame_;

    // init guess for ndt
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_cov_msg_;

    std::mutex ndt_map_mtx_;
    double converged_param_transform_probability_;
    std::map<std::string, std::string> key_value_stdmap_;
    
    void set_tf(
        const std::string & frame_id, const std::string & child_frame_id,
        const geometry_msgs::msg::PoseStamped & pose_msg);
};

std::unique_ptr<NdtLocalizer> new_operator(const NdtLocalizerConfig &cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(NdtLocalizer &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);
