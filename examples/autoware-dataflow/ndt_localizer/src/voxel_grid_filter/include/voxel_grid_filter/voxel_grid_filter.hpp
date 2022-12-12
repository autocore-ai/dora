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
#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

struct VoxelGridFilterConfig;

class VoxelGridFilter
{
public:
    VoxelGridFilter(const VoxelGridFilterConfig &cfg);
    ~VoxelGridFilter(){}
    sensor_msgs::msg::PointCloud2::ConstSharedPtr get_filtered_msg_ptr(){
        return std::make_shared<const sensor_msgs::msg::PointCloud2>(filtered_msg_);
    }
    void callback_scan(sensor_msgs::msg::PointCloud2::ConstSharedPtr& input);
private:
    double voxel_leaf_size_;
    bool output_log_;
    sensor_msgs::msg::PointCloud2 filtered_msg_;
    pcl::PointCloud<pcl::PointXYZ> removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range);
};

std::unique_ptr<VoxelGridFilter> new_operator(const VoxelGridFilterConfig &cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(VoxelGridFilter &op, rust::Str, rust::Slice<const uint8_t>, OutputSender &output_sender);
