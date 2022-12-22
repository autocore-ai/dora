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


#ifndef POINTCLOUD_DOWNSAMPLE__FILTERS__VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_
#define POINTCLOUD_DOWNSAMPLE__FILTERS__VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_

#include "pointcloud_downsample/cxx.h"

#include <boost/thread/mutex.hpp>
#include <sensor_msgs/msg/point_cloud2.h>

// PCL includes
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>

#include <vector>
#include <memory>
#include <string>

#include <stdio.h>

struct VoxelGridDownsampleFilterConfig;

class VoxelGridDownsampleFilter
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit VoxelGridDownsampleFilter(const VoxelGridDownsampleFilterConfig &cfg);
  ~VoxelGridDownsampleFilter(){}

  void filter(
    const PointCloud2ConstPtr & input, PointCloud2 & output);

private:
  inline bool isValid(
    const PointCloud2ConstPtr & cloud, const std::string & /*topic_name*/ = "input")
  {
    if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
      printf(
        "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with frame %s received!",
        cloud->data.size(), cloud->width, cloud->height, cloud->point_step, cloud->header.frame_id.c_str());
      return false;
    }
    return true;
  }

  double voxel_size_x_;
  double voxel_size_y_;
  double voxel_size_z_;

  /** \brief Internal mutex. */
  // std::mutex mutex_;
};

#endif  // POINTCLOUD_DOWNSAMPLE__FILTERS__VOXEL_GRID_DOWNSAMPLE_FILTER_HPP_
