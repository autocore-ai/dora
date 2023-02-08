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

#include "ndt_localizer/ndt_localizer.hpp"
#include "ndt_localizer/ffi.rs.h"
#include <iostream>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_sensor_msgs.hpp"
#include "cereal_ros_msgs/cereal_geometry_msgs.hpp"


NdtLocalizer::NdtLocalizer(const NdtLocalizerConfig &cfg) {
    key_value_stdmap_["state"] = "Initializing";
    base_frame_ = static_cast<std::string>(cfg.base_frame);
    map_frame_ = "map";

    double trans_epsilon = cfg.trans_epsilon;
    double step_size = cfg.step_size;
    double resolution = cfg.resolution;
    int max_iterations = cfg.max_iterations;

    ndt_.setTransformationEpsilon(trans_epsilon);
    ndt_.setStepSize(step_size);
    ndt_.setResolution(resolution);
    ndt_.setMaximumIterations(max_iterations);

    converged_param_transform_probability_ = cfg.converged_param_transform_probability;
}

void NdtLocalizer::callback_init_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & initial_pose_msg_ptr)
{
    if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
        initial_pose_cov_msg_ = *initial_pose_msg_ptr;
    } else {
        // // get TF from map_frame to pose_frame
        // geometry_msgs::msg::TransformStamped::SharedPtr TF_map2pose_ptr(new geometry_msgs::msg::TransformStamped);
        // get_transform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_map2pose_ptr);

        // // transform pose_frame to map_frame
        // geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr mapTF_initial_pose_msg_ptr(
        // new geometry_msgs::msg::PoseWithCovarianceStamped);
        // tf2::doTransform(*initial_pose_msg_ptr, *mapTF_initial_pose_msg_ptr, *TF_map2pose_ptr);
        // initial_pose_cov_msg_ = *mapTF_initial_pose_msg_ptr;
    }
    // if click the initpose again, re init！
    init_pose = false;
}

void NdtLocalizer::callback_map_points(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr & map_points_msg_ptr)
{
    const auto trans_epsilon = ndt_.getTransformationEpsilon();
    const auto step_size = ndt_.getStepSize();
    const auto resolution = ndt_.getResolution();
    const auto max_iterations = ndt_.getMaximumIterations();

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;

    ndt_new.setTransformationEpsilon(trans_epsilon);
    ndt_new.setStepSize(step_size);
    ndt_new.setResolution(resolution);
    ndt_new.setMaximumIterations(max_iterations);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
    ndt_new.setInputTarget(map_points_ptr);
    // create Thread
    // detach
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt_new.align(*output_cloud, Eigen::Matrix4f::Identity());

    // swap
    ndt_map_mtx_.lock();
    ndt_ = ndt_new;
    ndt_map_mtx_.unlock();
}

void NdtLocalizer::callback_tf_baselink2lidar(geometry_msgs::msg::TransformStamped::ConstSharedPtr & TF_msg_ptr)
{
    TF_baselink2lidar_ptr_ = TF_msg_ptr;
}

void NdtLocalizer::callback_pointcloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr & sensor_points_sensorTF_msg_ptr)
{
    const auto exe_start_time = std::chrono::system_clock::now();
    // mutex Map
    std::lock_guard<std::mutex> lock(ndt_map_mtx_);

    const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
    const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
    // get TF base to sensor
    // geometry_msgs::msg::TransformStamped::SharedPtr TF_base_to_sensor_ptr(new geometry_msgs::msg::TransformStamped);
    // get_transform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);

    // const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
    if (!TF_baselink2lidar_ptr_) {
        std::cout << "transform from baselink to lidar is empty, waiting for this message..." << std::endl;
        return;
    }
    const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_baselink2lidar_ptr_);
    const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(
        *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);
    
    // set input point cloud
    ndt_.setInputSource(sensor_points_baselinkTF_ptr);

    if (ndt_.getInputTarget() == nullptr) {
        std::cout << "NO MAP! "<< std::endl;
        return;
    }
    // align
    Eigen::Matrix4f initial_pose_matrix;
    if (!init_pose){
        Eigen::Affine3d initial_pose_affine;
        tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
        initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
        // for the first time, we don't know the pre_trans, so just use the init_trans, 
        // which means, the delta trans for the second time is 0
        pre_trans = initial_pose_matrix;
        init_pose = true;
    }else
    {
        // use predicted pose as init guess (currently we only impl linear model)
        initial_pose_matrix = pre_trans * delta_trans;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const auto align_start_time = std::chrono::system_clock::now();
    key_value_stdmap_["state"] = "Aligning";
    ndt_.align(*output_cloud, initial_pose_matrix);
    key_value_stdmap_["state"] = "Sleeping";
    const auto align_end_time = std::chrono::system_clock::now();
    const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() /1000.0;

    const Eigen::Matrix4f result_pose_matrix = ndt_.getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::msg::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

    const auto exe_end_time = std::chrono::system_clock::now();
    const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

    const float transform_probability = ndt_.getTransformationProbability();
    const int iteration_num = ndt_.getFinalNumIteration();

    is_converged = true;
    static size_t skipping_publish_num = 0;
    if (
        iteration_num >= ndt_.getMaximumIterations() + 2 ||
        transform_probability < converged_param_transform_probability_) {
        is_converged = false;
        ++skipping_publish_num;
        std::cout << "Not Converged" << std::endl;
    } else {
        skipping_publish_num = 0;
    }
    // calculate the delta tf from pre_trans to current_trans
    delta_trans = pre_trans.inverse() * result_pose_matrix;

    Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
    std::cout<<"delta x: "<<delta_translation(0) << " y: "<<delta_translation(1)<<
                " z: "<<delta_translation(2)<<std::endl;

    Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
    Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2,1,0);
    std::cout<<"delta yaw: "<<delta_euler(0) << " pitch: "<<delta_euler(1)<<
                " roll: "<<delta_euler(2)<<std::endl;

    pre_trans = result_pose_matrix;
    
    // publish
    if (is_converged) {
        ndt_pose_msg_.header.stamp = sensor_ros_time;
        ndt_pose_msg_.header.frame_id = map_frame_;
        ndt_pose_msg_.pose = result_pose_msg;
    }

    // publish tf(map frame to base frame)
    set_tf(map_frame_, base_frame_, ndt_pose_msg_);

    // publish aligned point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(
        *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_aligned_pose_msg_);
    sensor_aligned_pose_msg_.header.stamp = sensor_ros_time;
    sensor_aligned_pose_msg_.header.frame_id = map_frame_;

    exe_time_msg_.data = exe_time;
    transform_probability_msg_.data = transform_probability;
    iteration_num_msg_.data = iteration_num;

    key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
    key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
    key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "align_time: " << align_time << "ms" << std::endl;
    std::cout << "exe_time: " << exe_time << "ms" << std::endl;
    std::cout << "trans_prob: " << transform_probability << std::endl;
    std::cout << "iter_num: " << iteration_num << std::endl;
    std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;
}

void NdtLocalizer::set_tf(
    const std::string & frame_id, const std::string & child_frame_id,
    const geometry_msgs::msg::PoseStamped & pose_msg)
{
    TF_map2baselink_msg_.header.frame_id = frame_id;
    TF_map2baselink_msg_.child_frame_id = child_frame_id;
    TF_map2baselink_msg_.header.stamp = pose_msg.header.stamp;

    TF_map2baselink_msg_.transform.translation.x = pose_msg.pose.position.x;
    TF_map2baselink_msg_.transform.translation.y = pose_msg.pose.position.y;
    TF_map2baselink_msg_.transform.translation.z = pose_msg.pose.position.z;

    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
    TF_map2baselink_msg_.transform.rotation.x = tf_quaternion.x();
    TF_map2baselink_msg_.transform.rotation.y = tf_quaternion.y();
    TF_map2baselink_msg_.transform.rotation.z = tf_quaternion.z();
    TF_map2baselink_msg_.transform.rotation.w = tf_quaternion.w();
}


std::unique_ptr<NdtLocalizer> new_operator(const NdtLocalizerConfig &cfg)
{
    return std::make_unique<NdtLocalizer>(cfg);
}

OnInputResult on_input(NdtLocalizer &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
    // input process.  
    // std::cout << "Ndt_localizer operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;
    
    // deseralize
    std::stringstream ss; // any(in/out) stream can be used
    copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));
    if (id == "initial_pose") {
        geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr init_pose_ptr;
        {
            cereal::PortableBinaryInputArchive iarchive(ss); // Create an input archive
            iarchive(init_pose_ptr); // Read the data from the archive
        }
        // if (init_pose_ptr == nullptr) { 
        //     std::cerr << __FUNCTION__ << ": init_pose_ptr is null pointer" << std::endl;
        // }
        op.callback_init_pose(init_pose_ptr);
    } else if (id == "map_points") {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_ptr;
        {
            cereal::PortableBinaryInputArchive iarchive(ss);
            iarchive(map_points_ptr);
        }
        op.callback_map_points(map_points_ptr);
    } else if (id == "tf_baselink2lidar") {
        geometry_msgs::msg::TransformStamped::ConstSharedPtr TF_baselink2lidar_ptr;
        {
            cereal::PortableBinaryInputArchive iarchive(ss);
            iarchive(TF_baselink2lidar_ptr);
        }
        op.callback_tf_baselink2lidar(TF_baselink2lidar_ptr);
    } else {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_ptr;
        {
            cereal::PortableBinaryInputArchive iarchive(ss);
            iarchive(pointcloud_ptr);
        }
        op.callback_pointcloud(pointcloud_ptr);

        // output construct
        ss.str(""); // clear the buffer of ss
        {
            cereal::PortableBinaryOutputArchive oarchive(ss);
            oarchive(op.get_transform_probability_msg_ptr());
        }
        std::string str = ss.str();
        // string --> unsigned char --> rust::Slice 
        auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
        rust::Slice<const uint8_t> out_slice_trans_prob{uchar.data(), uchar.size()};
        auto send_result_trans_prob = send_output(output_sender, rust::Str("transform_porbability"), out_slice_trans_prob);
        OnInputResult result_trans_prob = {send_result_trans_prob.error, false};

        ss.str("");
        {
            cereal::PortableBinaryOutputArchive oarchive(ss);
            oarchive(op.get_iteration_num_msg_ptr());
        }
        str = ss.str();
        uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
        rust::Slice<const uint8_t> out_slice_iter_num{uchar.data(), uchar.size()};
        auto send_result_iter_num = send_output(output_sender, rust::Str("iteration_num"), out_slice_iter_num);
        OnInputResult result_iter_num = {send_result_iter_num.error, false};

        ss.str("");
        {
            cereal::PortableBinaryOutputArchive oarchive(ss);
            oarchive(op.get_tf_map2baselink_ptr());
        }
        str = ss.str();
        uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
        rust::Slice<const uint8_t> out_slice_tf_map2baselink{uchar.data(), uchar.size()};
        auto send_result_tf_map2baselink = send_output(output_sender, rust::Str("tf_map2baselink"), out_slice_tf_map2baselink);
        OnInputResult result_tf_map2baselink = {send_result_tf_map2baselink.error, false};

        ss.str("");
        {
            cereal::PortableBinaryOutputArchive oarchive(ss); // Create an output archive
            oarchive(op.get_ndt_pose_msg_ptr()); // Write the data to the archive
        }
        if (op.get_is_converged()){
            str = ss.str();
            uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
            rust::Slice<const uint8_t> out_slice_pose{uchar.data(), uchar.size()};
            auto send_result_pose = send_output(output_sender, rust::Str("ndt_pose"), out_slice_pose);
            OnInputResult result_pose = {send_result_pose.error, false};
            return result_pose;
        }
    }
    // Set the default return, which indicates there are two possibilities.
    // First: receive an input among [init_pose, map, tf_baselink2lidar].
    // Second: receive the pointcloud input, but ndt is not converged.
    return OnInputResult{rust::String(""), false};
}
