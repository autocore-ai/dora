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

#include "vehicle_twist/lib.rs.h"

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <time_utils/time_utils.hpp>

#include <iostream>
#include <vector>
#include <memory>
#include <string>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include <cereal_ros_msgs/cereal_geometry_msgs.hpp>


int main()
{
    auto dora_node = init_dora_node();
    // input process
    auto input = next_input(dora_node.inputs);
    if (input.end_of_input){
        return 0;
    }
    // std::cout << "Vehicle_twist node received input " << std::string(input.id) << std::endl;

    // Construct twist_with_covariance with: 
    double stddev_vx_ = 0.2;    // [m/s]
    double stddev_wz_ = 0.1;    // [rad/s]
    geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance_msg;

    twist_with_covariance_msg.header.frame_id = "base_link";
    twist_with_covariance_msg.header.stamp = time_utils::to_message(std::chrono::system_clock::now());

    twist_with_covariance_msg.twist.twist.linear.x = 0.0;
    twist_with_covariance_msg.twist.twist.linear.y = 0.0;
    twist_with_covariance_msg.twist.twist.angular.z = 0.0;
    twist_with_covariance_msg.twist.covariance[0 + 0 * 6] = stddev_vx_ * stddev_vx_;
    twist_with_covariance_msg.twist.covariance[1 + 1 * 6] = 10000.0;
    twist_with_covariance_msg.twist.covariance[2 + 2 * 6] = 10000.0;
    twist_with_covariance_msg.twist.covariance[3 + 3 * 6] = 10000.0;
    twist_with_covariance_msg.twist.covariance[4 + 4 * 6] = 10000.0;
    twist_with_covariance_msg.twist.covariance[5 + 5 * 6] = stddev_wz_ * stddev_wz_;

    auto twist_with_covariance_ptr = std::make_shared<const geometry_msgs::msg::TwistWithCovarianceStamped>(twist_with_covariance_msg);
    
    while (true) {
        // seralize the msg to raw data (e.g. [u8] or Vec<u8>) as following
        std::stringstream ss; // any(in/out) stream can be used
        {
            cereal::PortableBinaryOutputArchive oarchive(ss); // Create an output archive
            oarchive(twist_with_covariance_ptr);              // Write the data to the archive
        } // archive goes out of scope, ensuring all contents are flushed
        std::string str = ss.str();
        // string --> unsigned char --> rust::Slice 
        auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
        rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
        
        // send data
        auto result = send_output(dora_node.send_output, "twist_with_cov", out_slice);
        auto error = std::string(result.error);
        if (!error.empty()){
            std::cerr << "Error: " << error << std::endl;
            return -1;
        }
    }
    return 0;
}