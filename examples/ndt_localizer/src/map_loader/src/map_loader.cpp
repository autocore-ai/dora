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

#include "map_loader/lib.rs.h"

#include <iostream>
#include <filesystem>
#include <vector>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include <cereal_ros_msgs/cereal_sensor_msgs.hpp>

namespace fs = std::filesystem;

bool isPcdFile(const std::string & p)
{
  if (fs::is_directory(p)) {
    return false;
  }

  const std::string ext = fs::path(p).extension();
  if (ext != ".pcd" && ext != ".PCD") {
    return false;
  }

  return true;
}

sensor_msgs::msg::PointCloud2 CreatePcd(const std::vector<std::string> & pcd_paths){
    sensor_msgs::msg::PointCloud2 whole_pcd{};

    sensor_msgs::msg::PointCloud2 partial_pcd;
    for (const auto & path : pcd_paths) {
        if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
            std::cerr << "PCD load failed: " << path << std::endl;
        }
        if (whole_pcd.width == 0) {
            whole_pcd = partial_pcd;
        } else {
            whole_pcd.width += partial_pcd.width;
            whole_pcd.row_step += partial_pcd.row_step;
            whole_pcd.data.reserve(whole_pcd.data.size() + partial_pcd.data.size());
            whole_pcd.data.insert(whole_pcd.data.end(), partial_pcd.data.begin(), partial_pcd.data.end());
        }
    }

    whole_pcd.header.frame_id = "map";
    return whole_pcd;
}


int main()
{
    auto dora_node = init_dora_node();
    // input process
    auto input = next_input(dora_node.inputs);
    if (input.end_of_input){
        return 0;
    }
    // std::cout << "Map_loader node received input " << std::string(input.id) << std::endl;
    
    for (int i = 0; i < 2; i++) {
        // load pcd
        std::string map_path_or_dir = fs::absolute("src/map_loader/map").string();
        std::vector<std::string> pcd_paths{};
        if (!fs::exists(map_path_or_dir)) {
            std::cerr<<"invalid map path: " << map_path_or_dir <<std::endl;
        }
        if (isPcdFile(map_path_or_dir)) {
            pcd_paths.push_back(map_path_or_dir);
        }
        if (fs::is_directory(map_path_or_dir)) {
            for (const auto & file : fs::directory_iterator(map_path_or_dir)) {
                const auto filename = file.path().string();
                if (isPcdFile(filename)) {
                    pcd_paths.push_back(filename);
                }
            }
        }
        auto pc_msg = CreatePcd(pcd_paths);
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
        auto result = send_output(dora_node.send_output, "map_points", out_slice);
        auto error = std::string(result.error);
        if (!error.empty()){
            std::cerr << "Error: " << error << std::endl;
            return -1;
        }
    }
    return 0;
}
