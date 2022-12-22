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


#include "pointcloud_downsample/pointcloud_downsample.hpp"
#include "pointcloud_downsample/ffi.rs.h"

#include "pointcloud_downsample/filters/crop_box_filter.hpp"
#include "pointcloud_downsample/filters/voxel_grid_downsample_filter.hpp"
#include "pointcloud_downsample/filters/random_downsample_filter.hpp"

#include <time_utils/time_utils.hpp>

#include <pcl_ros/transforms.hpp>

// #include <cereal/archives/binary.hpp>
#include "cereal/archives/portable_binary.hpp"
#include "cereal/types/memory.hpp"
#include <sstream>
#include "cereal_ros_msgs/cereal_sensor_msgs.hpp"


PointcloudDownsample::PointcloudDownsample(
  const PointcloudDownsampleConfig &downsample_cfg,
  const CropBoxFilterConfig &crop_box_cfg, 
  const VoxelGridDownsampleFilterConfig &voxel_grid_downsample_cfg, 
  const RandomDownsampleFilterConfig random_downsample_cfg)
{
  tf_input_frame_ = static_cast<std::string>(downsample_cfg.input_frame);
  tf_output_frame_ = static_cast<std::string>(downsample_cfg.output_frame);

  crop_box_ = std::make_unique<CropBoxFilter>(crop_box_cfg);
  voxel_grid_downsample_ = std::make_unique<VoxelGridDownsampleFilter>(voxel_grid_downsample_cfg);
  random_downsample_ = std::make_unique<RandomDownsampleFilter>(random_downsample_cfg);
}

PointcloudDownsample::~PointcloudDownsample() = default;

void PointcloudDownsample::filter(const PointCloud2ConstPtr &input)
{
  // std::scoped_lock lock(mutex_);
  PointCloud2 crop_box_output;
  crop_box_->filter(input, crop_box_output);

  PointCloud2 voxel_grid_downsample_output;
  voxel_grid_downsample_->filter(
    std::make_shared<const PointCloud2>(crop_box_output),
    voxel_grid_downsample_output);

  random_downsample_->filter(
    std::make_shared<const PointCloud2>(voxel_grid_downsample_output), 
    downsampled_pointcloud_msg_);
}


std::unique_ptr<PointcloudDownsample> new_operator(
  const PointcloudDownsampleConfig &downsample_cfg,
  const CropBoxFilterConfig &crop_box_cfg,
  const VoxelGridDownsampleFilterConfig & voxel_grid_downsample_cfg,
  const RandomDownsampleFilterConfig & random_downsample_cfg)
{
  return std::make_unique<PointcloudDownsample>(
    downsample_cfg, crop_box_cfg, voxel_grid_downsample_cfg, random_downsample_cfg);
}

OnInputResult on_input(
  PointcloudDownsample &op, rust::Str id, 
  rust::Slice<const uint8_t> data, OutputSender &output_sender)
{
  // input process.  
  // std::cout << "PointcloudDownsample operator received input `" << id << "` with data `" << (unsigned int)data[0] << std::endl;
  
  // deseralize
  std::stringstream ss; // any(in/out) stream can be used
  copy(data.begin(), data.end(), std::ostream_iterator<unsigned char>(ss,""));
  
  sensor_msgs::msg::PointCloud2::ConstSharedPtr raw_pointcloud_ptr;
  {
    cereal::PortableBinaryInputArchive iarchive(ss);
    iarchive(raw_pointcloud_ptr);
  }
  op.filter(raw_pointcloud_ptr);

  // output construct
  ss.clear();
  {
    cereal::PortableBinaryOutputArchive oarchive(ss);
    oarchive(op.get_downsampled_pointcloud_msg_ptr());
  }
  std::string str = ss.str();
  // string --> unsigned char --> rust::Slice 
  auto uchar = std::vector<unsigned char>(str.data(), str.data()+str.size()+1);
  rust::Slice<const uint8_t> out_slice{uchar.data(), uchar.size()};
  auto send_result = send_output(output_sender, rust::Str("downsampled_pointcloud"), out_slice);
  OnInputResult result = {send_result.error, false};
  return result;
}
