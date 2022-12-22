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


#ifndef POINTCLOUD_DOWNSAMPLE__POINTCLOUD_DOWNSAMPLE_HPP_
#define POINTCLOUD_DOWNSAMPLE__POINTCLOUD_DOWNSAMPLE_HPP_

#include "cxx.h"

#include <sensor_msgs/msg/point_cloud2.h>
// PCL includes
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <vector>
#include <memory>
#include <string>

struct PointcloudDownsampleConfig;
struct CropBoxFilterConfig;
struct VoxelGridDownsampleFilterConfig;
struct RandomDownsampleFilterConfig;

class CropBoxFilter;
class VoxelGridDownsampleFilter;
class RandomDownsampleFilter;

class PointcloudDownsample
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PointcloudDownsample(
    const PointcloudDownsampleConfig &downsample_cfg,
    const CropBoxFilterConfig &crop_box_cfg, 
    const VoxelGridDownsampleFilterConfig & voxel_grid_downsample_cfg,
    const RandomDownsampleFilterConfig random_downsample_cfg);
  ~PointcloudDownsample();

  PointCloud2ConstPtr get_downsampled_pointcloud_msg_ptr(){
    return std::make_shared<const PointCloud2>(downsampled_pointcloud_msg_);
  }

  void filter(const PointCloud2ConstPtr & input);

private:
  PointCloud2 downsampled_pointcloud_msg_;
  
  std::unique_ptr<CropBoxFilter> crop_box_;
  std::unique_ptr<VoxelGridDownsampleFilter> voxel_grid_downsample_;
  std::unique_ptr<RandomDownsampleFilter> random_downsample_;

  /** \brief Internal mutex. */
  // std::mutex mutex_;

  /** \brief The input TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_input_frame_;
  
  /** \brief The output TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_output_frame_;
};

// Take pointcloud downsample module as an example for show how to using namespace
std::unique_ptr<PointcloudDownsample> new_operator(
  const PointcloudDownsampleConfig &downsample_cfg,
  const CropBoxFilterConfig &crop_box_cfg,
  const VoxelGridDownsampleFilterConfig & voxel_grid_downsample_cfg,
  const RandomDownsampleFilterConfig & random_downsample_cfg);

struct OnInputResult;
struct OutputSender;

OnInputResult on_input(
  PointcloudDownsample &op, rust::Str, 
  rust::Slice<const uint8_t>, OutputSender &output_sender);

#endif  // POINTCLOUD_DOWNSAMPLE__POINTCLOUD_DOWNSAMPLE_HPP_
