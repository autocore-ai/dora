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


#ifndef POINTCLOUD_DOWNSAMPLE__FILTERS__CROP_BOX_FILTER_HPP_
#define POINTCLOUD_DOWNSAMPLE__FILTERS__CROP_BOX_FILTER_HPP_

#include "pointcloud_downsample/cxx.h"

#include <boost/thread/mutex.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>

// PCL includes
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <vector>
#include <memory>
#include <string>

#include <stdio.h>


struct CropBoxFilterConfig;

class CropBoxFilter
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit CropBoxFilter(const CropBoxFilterConfig &cfg);
  ~CropBoxFilter(){}

  void filter(
    const PointCloud2ConstPtr & input, PointCloud2 & output);

private:
  void setCropBoxPolygon();

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

  struct CropBoxParam
  {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    bool negative{false};
  } param_;

  /** \brief Internal mutex. */
  // std::mutex mutex_;
  /** \brief cal processing time. **/
  std::unique_ptr<tier4_autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  // additional publishers
  geometry_msgs::msg::PolygonStamped polygon_msg_;

  double cyclic_time_ms_;
  double processing_time_ms_;
};

#endif  // POINTCLOUD_DOWNSAMPLE__FILTERS__CROP_BOX_FILTER_HPP_
