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

#include "tf_publisher/lib.rs.h"

#include <iostream>
#include <memory>
#include <vector>

#include <time_utils/time_utils.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include <cereal_ros_msgs/cereal_geometry_msgs.hpp>


geometry_msgs::msg::TransformStamped get_transform(
    const std::string & frame_id, const std::string & child_frame_id)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.header.stamp = time_utils::to_message(std::chrono::system_clock::now());

    transform_stamped.transform.translation.x = 0.0;
    transform_stamped.transform.translation.y = 0.0;
    transform_stamped.transform.translation.z = 0.0;

    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(0.0, 0.0, 0.0);
    transform_stamped.transform.rotation.x = tf_quaternion.x();
    transform_stamped.transform.rotation.y = tf_quaternion.y();
    transform_stamped.transform.rotation.z = tf_quaternion.z();
    transform_stamped.transform.rotation.w = tf_quaternion.w();

    return transform_stamped;
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

    // while (true)
    for (int i = 0; i < 2; i++) {
        // publisher the transform from base_link to lidar (velodyne_top).
        std::string parent_frame = "base_link";
        std::string child_frame = "velodyne_top";
        auto transform_msg = get_transform(parent_frame, child_frame);

        auto transform_ptr = std::make_shared<const geometry_msgs::msg::TransformStamped>(transform_msg);
        // seralize the msg to raw data (e.g. [u8] or Vec<u8>) as following
        std::stringstream ss; // any(in/out) stream can be used
        {
            cereal::PortableBinaryOutputArchive oarchive(ss); // Create an output archive
            oarchive(transform_ptr);                          // Write the data to the archive
        } // archive goes out of scope, ensuring all contents are flushed
        std::string str = ss.str();
        // string --> unsigned char --> rust::Slice 
        auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
        rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
        
        // send data
        auto result = send_output(dora_node.send_output, "tf_baselink2lidar", out_slice);
        auto error = std::string(result.error);
        if (!error.empty()){
            std::cerr << "Error: " << error << std::endl;
            return -1;
        }
    }
    return 0;
}
