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


#include "pointcloud_downsample/filters/crop_box_filter.hpp"
#include "pointcloud_downsample/ffi.rs.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <time_utils/time_utils.hpp>

CropBoxFilter::CropBoxFilter(const CropBoxFilterConfig &cfg)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // set initial parameters
  {
    auto & p = param_;
    p.min_x = static_cast<float>(cfg.min_x);
    p.min_y = static_cast<float>(cfg.min_y);
    p.min_z = static_cast<float>(cfg.min_z);
    p.max_x = static_cast<float>(cfg.max_x);
    p.max_y = static_cast<float>(cfg.max_y);
    p.max_z = static_cast<float>(cfg.max_z);
    p.negative = static_cast<float>(cfg.negative);
  }
}

void CropBoxFilter::filter(
  const PointCloud2ConstPtr & input, PointCloud2 & output)
{
  if (!isValid(input)) {
    std::cerr << "[crop_box filter] Invalid input!" << std::endl;
    return;
  }

  // std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);
  output.data.resize(input->data.size());
  Eigen::Vector3f pt(Eigen::Vector3f::Zero());
  size_t j = 0;
  const auto data_size = input->data.size();
  const auto point_step = input->point_step;
  // If inside the cropbox
  if (!param_.negative) {
    for (size_t i = 0; i + point_step < data_size; i += point_step) {
      memcpy(pt.data(), &input->data[i], sizeof(float) * 3);
      if (
        param_.min_z < pt.z() && pt.z() < param_.max_z && param_.min_y < pt.y() &&
        pt.y() < param_.max_y && param_.min_x < pt.x() && pt.x() < param_.max_x) {
        memcpy(&output.data[j], &input->data[i], point_step);
        j += point_step;
      }
    }
    // If outside the cropbox
  } else {
    for (size_t i = 0; i + point_step < data_size; i += point_step) {
      memcpy(pt.data(), &input->data[i], sizeof(float) * 3);
      if (
        param_.min_z > pt.z() || pt.z() > param_.max_z || param_.min_y > pt.y() ||
        pt.y() > param_.max_y || param_.min_x > pt.x() || pt.x() > param_.max_x) {
        memcpy(&output.data[j], &input->data[i], point_step);
        j += point_step;
      }
    }
  }

  output.data.resize(j);
  output.header.frame_id = input->header.frame_id;
  output.height = 1;
  output.fields = input->fields;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.is_dense = input->is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);

  // Copy timestamp to keep it
  output.header.stamp = input->header.stamp;

  setCropBoxPolygon();
  // add processing time for debug
  cyclic_time_ms_ = stop_watch_ptr_->toc("cyclic_time", true);
  processing_time_ms_ = stop_watch_ptr_->toc("processing_time", true);
}

void CropBoxFilter::setCropBoxPolygon()
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  const double x1 = param_.max_x;
  const double x2 = param_.min_x;
  const double x3 = param_.min_x;
  const double x4 = param_.max_x;

  const double y1 = param_.max_y;
  const double y2 = param_.max_y;
  const double y3 = param_.min_y;
  const double y4 = param_.min_y;

  const double z1 = param_.min_z;
  const double z2 = param_.max_z;

  polygon_msg_.header.frame_id = "base_link";
  polygon_msg_.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  polygon_msg_.polygon.points.push_back(generatePoint(x1, y1, z1));
  polygon_msg_.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg_.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg_.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg_.polygon.points.push_back(generatePoint(x1, y1, z1));

  polygon_msg_.polygon.points.push_back(generatePoint(x1, y1, z2));

  polygon_msg_.polygon.points.push_back(generatePoint(x2, y2, z2));
  polygon_msg_.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg_.polygon.points.push_back(generatePoint(x2, y2, z2));

  polygon_msg_.polygon.points.push_back(generatePoint(x3, y3, z2));
  polygon_msg_.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg_.polygon.points.push_back(generatePoint(x3, y3, z2));

  polygon_msg_.polygon.points.push_back(generatePoint(x4, y4, z2));
  polygon_msg_.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg_.polygon.points.push_back(generatePoint(x4, y4, z2));

  polygon_msg_.polygon.points.push_back(generatePoint(x1, y1, z2));
}
