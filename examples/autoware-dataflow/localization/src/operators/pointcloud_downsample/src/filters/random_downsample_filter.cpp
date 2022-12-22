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


#include "pointcloud_downsample/filters/random_downsample_filter.hpp"
#include "pointcloud_downsample/ffi.rs.h"

#include <vector>

RandomDownsampleFilter::RandomDownsampleFilter(
  const RandomDownsampleFilterConfig &cfg)
{
  // set initial parameters
  {
    sample_num_ = static_cast<size_t>(cfg.sample_num);
  }
}

void RandomDownsampleFilter::filter(
  const PointCloud2ConstPtr & input, PointCloud2 & output)
{
  if (!isValid(input)) {
    std::cerr << "[random_downsample filter] Invalid input!" << std::endl;
    return;
  }

  // std::scoped_lock lock(mutex_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  pcl::RandomSample<pcl::PointXYZ> filter;
  filter.setInputCloud(pcl_input);
  // filter.setSaveLeafLayout(true);
  filter.setSample(sample_num_);
  filter.filter(*pcl_output);

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}
