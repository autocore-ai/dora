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

#include "voxel_grid_filter/voxel_grid_filter.hpp"
#include "voxel_grid_filter/ffi.rs.h"
#include <iostream>
#include <fstream>

#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_sensor_msgs.hpp"


#define MAX_MEASUREMENT_RANGE 120.0

VoxelGridFilter::VoxelGridFilter(const VoxelGridFilterConfig &cfg) {
    voxel_leaf_size_ = cfg.leaf_size;
    output_log_ = cfg.output_log;
    std::cout << "Voxel leaf size is: " << voxel_leaf_size_ << std::endl;
    if(output_log_ == true){
        char buffer[80];
        std::time_t now = std::time(NULL);
        std::tm *pnow = std::localtime(&now);
        std::strftime(buffer,80,"%Y%m%d_%H%M%S",pnow);
        std::string filename = "voxel_grid_filter_" + std::string(buffer) + ".csv";
        std::ofstream ofs;
        ofs.open(filename.c_str(), std::ios::app);
    }
}

pcl::PointCloud<pcl::PointXYZ> VoxelGridFilter::removePointsByRange(
    pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZ> narrowed_scan;
    narrowed_scan.header = scan.header;

    if( min_range>=max_range ) {
        std::cout << "min_range "<< min_range << ">= max_range: " << max_range << std::endl;
        return scan;
    }

    double square_min_range = min_range * min_range;
    double square_max_range = max_range * max_range;

    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = scan.begin(); iter != scan.end(); ++iter)
    {
        const pcl::PointXYZ &p = *iter;
        double square_distance = p.x * p.x + p.y * p.y;

        if(square_min_range <= square_distance && square_distance <= square_max_range){
            narrowed_scan.points.push_back(p);
        }
    }

    return narrowed_scan;
}

void VoxelGridFilter::callback_scan(sensor_msgs::msg::PointCloud2::ConstSharedPtr& input){
    pcl::PointCloud<pcl::PointXYZ> scan;
    pcl::fromROSMsg(*input, scan);
    scan = removePointsByRange(scan, 0, MAX_MEASUREMENT_RANGE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    if (voxel_leaf_size_ >= 0.1)
    {
        // Downsampling the velodyne scan using VoxelGrid filter
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
        voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_grid_filter.setInputCloud(scan_ptr);
        voxel_grid_filter.filter(*filtered_scan_ptr);
        pcl::toROSMsg(*filtered_scan_ptr, filtered_msg_);
    }
    else
    {
        pcl::toROSMsg(*scan_ptr, filtered_msg_);
    }

    filtered_msg_.header = input->header;
}


std::unique_ptr<VoxelGridFilter> new_operator(const VoxelGridFilterConfig &cfg)
{
    return std::make_unique<VoxelGridFilter>(cfg);
}

OnInputResult on_input(VoxelGridFilter &op, rust::Str id, rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
    // Input process. 
    // std::cout << "Voxel_gird_filter operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;

    std::stringstream ss; // any(in/out) stream can be used
    sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_ptr;
    copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));
    {
        cereal::PortableBinaryInputArchive iarchive(ss); // Create an input archive
        iarchive(pc_ptr); // Read the data from the archive
    }
    op.callback_scan(pc_ptr);

    // output construct. convert the msg to raw data(e.g. [u8] or Vec<u8>)
    ss.clear();
    {
        cereal::PortableBinaryOutputArchive oarchive(ss); // Create an output archive
        oarchive(op.get_filtered_msg_ptr()); // Write the data to the archive
    }
    // string --> unsigned char --> rust::Slice 
    std::string str = ss.str();
    auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
    rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};

    auto send_result = send_output(output_sender, rust::Str("filted_pointcloud"), out_slice);
    OnInputResult result = {send_result.error, false};
    return result;
}
