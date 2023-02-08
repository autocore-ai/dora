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

#include "gnss_imu_dummy/lib.rs.h"

#include <iostream>
#include <memory>
#include <vector>

#include <time_utils/time_utils.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include <cereal_ros_msgs/cereal_sensor_msgs.hpp>
// #include <cereal_ros_msgs/cereal_geometry_msgs.hpp>
#include <cereal_ros_msgs/cereal_autoware_sensing_msgs.hpp>


sensor_msgs::msg::NavSatFix::ConstSharedPtr get_gnss_msg_ptr(std::string & frame_id)
{
    sensor_msgs::msg::NavSatFix msg;
    msg.header.frame_id = frame_id; // set "imu_link" if using pbox else set "gnss_link"
    msg.header.stamp = time_utils::to_message(std::chrono::system_clock::now());

    // set according to the info provide by (https://map.jiqrxx.com/jingweidu/)
    msg.latitude = 31.71483392;
    msg.longitude = 118.97958934;
    msg.altitude = 28.159;
    // Position covariance [m^2], ENU
    double sh = 0.016;
    double sv = 0.024;
    msg.position_covariance = {sh * sh, 0, 0, 0, sh * sh, 0, 0, 0, sv * sv};
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = 0; // unknown

    // return msg;
    return std::make_shared<const sensor_msgs::msg::NavSatFix>(msg);
}

sensor_msgs::msg::Imu::ConstSharedPtr get_imu_msg_ptr(std::string & frame_id)
{
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id; // default "imu_link"
    msg.header.stamp = time_utils::to_message(std::chrono::system_clock::now());

    msg.orientation.w = 0.125;   // TODO: may need to be adjust
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = -0.992;
    // Orientation stddev: [0.0, 0.0, 0.0] [rad]
    msg.orientation_covariance[0] = 0.0;
    msg.orientation_covariance[4] = 0.0;
    msg.orientation_covariance[8] = 0.0;

    msg.angular_velocity.x = 0.0;
    msg.angular_velocity.y = 0.0;
    msg.angular_velocity.z = 0.0;
    // Angular velocity stddev: [0.0, 0.0, 0.0] [rad/s]
    msg.angular_velocity_covariance[0] = 0.0;
    msg.angular_velocity_covariance[4] = 0.0;
    msg.angular_velocity_covariance[8] = 0.0;

    msg.linear_acceleration.x = 0.0;
    msg.linear_acceleration.y = 0.0;
    msg.linear_acceleration.z = 0.0;
    // Linear_acceleration_stddev: [0.0, 0.0, 0.0] # [m/s^2]
    msg.linear_acceleration_covariance[0] = 0.0;
    msg.linear_acceleration_covariance[4] = 0.0;
    msg.linear_acceleration_covariance[8] = 0.0;

    // return msg;
    return std::make_shared<const sensor_msgs::msg::Imu>(msg);
}

/*
geometry_msgs::msg::QuaternionStamped::ConstSharedPtr get_orientation_msg_ptr(std::string & frame_id)
{
    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.frame_id = frame_id; // // set "imu_link" if using pbox else set "gnss_link"
    msg.header.stamp = time_utils::to_message(std::chrono::system_clock::now());

    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(0.0, 0.0, -2.8905903);    // may need to be adjust

    msg.quaternion.w = tf_quaternion.w();
    msg.quaternion.x = tf_quaternion.x();
    msg.quaternion.y = tf_quaternion.y();
    msg.quaternion.z = tf_quaternion.z();

    // return msg;
    return std::make_shared<const geometry_msgs::msg::QuaternionStamped>(msg);
}
*/

autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr get_orientation_msg_ptr(std::string & frame_id)
{
    autoware_sensing_msgs::msg::GnssInsOrientationStamped msg;
    msg.header.frame_id = frame_id; // // set "imu_link" if using pbox else set "gnss_link"
    msg.header.stamp = time_utils::to_message(std::chrono::system_clock::now());

    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(0.0, 0.0, -2.8905903);    // may need to be adjust

    msg.orientation.orientation.w = tf_quaternion.w();
    msg.orientation.orientation.x = tf_quaternion.x();
    msg.orientation.orientation.y = tf_quaternion.y();
    msg.orientation.orientation.z = tf_quaternion.z();

    msg.orientation.rmse_rotation_x = 0.32;
    msg.orientation.rmse_rotation_y = 0.32;
    msg.orientation.rmse_rotation_y = 1.0;

    // return msg;
    return std::make_shared<const autoware_sensing_msgs::msg::GnssInsOrientationStamped>(msg);
}

int main()
{
    auto dora_node = init_dora_node();
    // input process
    auto input = next_input(dora_node.inputs);
    if (input.end_of_input){
        return 0;
    }
    // std::cout << "TF_publisher node received input " << std::string(input.id) << std::endl;

    std::string gnss_frame = "gnss_link";
    std::string imu_frame = "imu_link";
    while (true) {
        // publisher the transform.
        auto gnss_msg_ptr = get_gnss_msg_ptr(gnss_frame);
        auto imu_msg_ptr = get_imu_msg_ptr(imu_frame);
        auto orientation_msg_ptr = get_orientation_msg_ptr(gnss_frame);

        // seralize the msg to raw data (e.g. [u8] or Vec<u8>) as following
        std::stringstream ss; // any(in/out) stream can be used
        {
            cereal::PortableBinaryOutputArchive oarchive(ss);   // Create an output archive
            oarchive(gnss_msg_ptr);                       // Write the data to the archive
        } // archive goes out of scope, ensuring all contents are flushed
        std::string str = ss.str();
        // string --> unsigned char --> rust::Slice 
        auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
        rust::Slice<const uint8_t> out_slice_gnss{uchar.data(), uchar.size()};
        // send data
        auto result = send_output(dora_node.send_output, "nav_sat_fix", out_slice_gnss);
        auto error = std::string(result.error);
        if (!error.empty()){
            std::cerr << "Error: " << error << std::endl;
            return -1;
        }

        ss.str(""); // clear the buffer of ss
        {
            cereal::PortableBinaryOutputArchive oarchive(ss);
            oarchive(imu_msg_ptr);
        }
        str = ss.str();
        uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
        rust::Slice<const uint8_t> out_slice_imu{uchar.data(), uchar.size()};
        result = send_output(dora_node.send_output, "imu_raw", out_slice_imu);
        error = std::string(result.error);
        if (!error.empty()){
            std::cerr << "Error: " << error << std::endl;
            return -1;
        }

        ss.str("");
        {
            cereal::PortableBinaryOutputArchive oarchive(ss);
            oarchive(orientation_msg_ptr);
        }
        str = ss.str();
        uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
        rust::Slice<const uint8_t> out_slice_orientation{uchar.data(), uchar.size()};
        result = send_output(dora_node.send_output, "orientation", out_slice_orientation);
        error = std::string(result.error);
        if (!error.empty()){
            std::cerr << "Error: " << error << std::endl;
            return -1;
        }
    }
    return 0;
}
