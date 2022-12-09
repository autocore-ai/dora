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

/// dummy of rosbag play tool, using temporarily
#include "rosbag_dummy/lib.rs.h"

#include <iostream>
#include <filesystem>
#include <vector>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <time_utils/time_utils.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include <cereal_ros_msgs/cereal_sensor_msgs.hpp>

namespace fs = std::filesystem;

auto file_name_iter = fs::directory_iterator(fs::absolute("src/rosbag_dummy/pcd_files").string());

bool get_next_pcd(const std::string & pcd_file_name, sensor_msgs::msg::PointCloud2 & out_msg)
{
    std::cout << "Pcd file name: "<< pcd_file_name << std::endl;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_name, cloud)== -1)
    {
        std::cerr << "No target pcd file" << std::endl;
        return false;
    }
    pcl::toROSMsg(cloud, out_msg);
    out_msg.header.frame_id = "velodyne_top";
    out_msg.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
    file_name_iter++;
    return true;
}

int main()
{
    auto dora_node = init_dora_node();
    // input process
    auto input = next_input(dora_node.inputs);
    if (input.end_of_input){
        return 0;
    }
    std::cout << "Rosbag_dummy node received input " << std::string(input.id) << std::endl;

    while (true) {
        // read a pcd file from certain directory and send it out
        bool flag;
        sensor_msgs::msg::PointCloud2 pc_msg;
        if (file_name_iter != fs::directory_iterator()){
            flag = get_next_pcd((*file_name_iter).path(), pc_msg);
            if (flag == false) {
                return -1;  // it will not trigger 
            }
        } else{
            std::cout << "Rosbag end reached " << std::endl;
            return 0;
        }

        auto pc_ptr = std::make_shared<const sensor_msgs::msg::PointCloud2>(pc_msg);
        // seralize the msg to raw data (e.g. [u8] or Vec<u8>) as following
        std::stringstream ss; // any(in/out) stream can be used
        {
            cereal::PortableBinaryOutputArchive oarchive(ss); // Create an output archive
            oarchive(pc_ptr);                                 // Write the data to the archive
        } // archive goes out of scope, ensuring all contents are flushed
        std::string str = ss.str();
        // string --> unsigned char --> rust::Slice 
        auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
        rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
        
        // send data
        auto result = send_output(dora_node.send_output, "pointcloud", out_slice);
        auto error = std::string(result.error);
        if (!error.empty()){
            std::cerr << "Error: " << error << std::endl;
            return -1;
        }
    }
    return 0;
}