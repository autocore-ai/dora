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

#include "manual_pose/lib.rs.h"

#include <iostream>
#include <vector>
#include <memory>

#include <time_utils/time_utils.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    // std::cout << "manual_pose node received input " << std::string(input.id) << std::endl;

    // construct initial pose
    std::array<double, 36> pose_covariance {
        1.0, 0.0, 0.0,  0.0,  0.0,  0.0,
        0.0, 1.0, 0.0,  0.0,  0.0,  0.0,
        0.0, 0.0, 0.01, 0.0,  0.0,  0.0,
        0.0, 0.0, 0.0,  0.01, 0.0,  0.0,
        0.0, 0.0, 0.0,  0.0,  0.01, 0.0,
        0.0, 0.0, 0.0,  0.0,  0.0,  0.2,
    };
    // geometry_msgs::msg::Vector3 xyz(-6.865, -1.397, -1.906);
    // geometry_msgs::msg::Vector3 rpy(0.0, 0.0, -2.88);    // the corresponding quaternion xyzw (0.0, 0.0, -0.992, 0.125)
    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(0.0, 0.0, -2.88);
    geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;

    // for (int i = 0; i < 2; i++) {
        init_pose_msg.header.frame_id = "map";
        init_pose_msg.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
        init_pose_msg.pose.pose.position.x = -6.865;
        init_pose_msg.pose.pose.position.y = -1.397;
        init_pose_msg.pose.pose.position.z = -1.906;
        init_pose_msg.pose.pose.orientation = tf2::toMsg(tf_quaternion);
        init_pose_msg.pose.covariance = pose_covariance;

        auto init_pose_ptr = std::make_shared<const geometry_msgs::msg::PoseWithCovarianceStamped>(init_pose_msg);
    
        // seralize the msg to raw data (e.g. [u8] or Vec<u8>) as following
        std::stringstream ss; // any(in/out) stream can be used
        {
            cereal::PortableBinaryOutputArchive oarchive(ss); // Create an output archive
            oarchive(init_pose_ptr);                                 // Write the data to the archive
        } // archive goes out of scope, ensuring all contents are flushed
        std::string str = ss.str();
        // string --> unsigned char --> rust::Slice 
        auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
        rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
        
        // send data
        auto result = send_output(dora_node.send_output, "initial_pose", out_slice);
        auto error = std::string(result.error);
        if (!error.empty()){
            std::cerr << "Error: " << error << std::endl;
            return -1;
        }
    // }
    return 0;
}
