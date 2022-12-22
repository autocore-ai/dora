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


#include "pointcloud_downsample/filters/filter.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl/io/io.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <iostream>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::computePublish(
  const PointCloud2ConstPtr & input)
{
  auto output = std::make_unique<PointCloud2>();
  // Call the virtual method in the child
  filter(input, *output);
  // Copy timestamp to keep it
  output->header.stamp = input->header.stamp;
}


//////////////////////////////////////////////////////////////////////////////////////////////
void pointcloud_preprocessor::Filter::input_indices_callback(
  const PointCloud2ConstPtr cloud)
{
  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    std::cerr << "[input_indices_callback] Invalid input!" << std::endl;
    return;
  }

  computePublish(cloud);
}
