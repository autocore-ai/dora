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


#ifndef POINTCLOUD_DOWNSAMPLE__FILTERS__FILTER_HPP_
#define POINTCLOUD_DOWNSAMPLE__FILTERS__FILTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <stdio.h>

#include <boost/thread/mutex.hpp>
#include <sensor_msgs/msg/point_cloud2.h>
// PCL includes
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Include tier4 autoware utils
#include <tier4_autoware_utils/system/stop_watch.hpp>

/** \brief @b Filter represents the base filter class. */
class Filter
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit Filter(){}
  ~Filter(){}

protected:
  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** \brief processing time publisher. **/
  std::unique_ptr<tier4_autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  /** \brief Virtual abstract filter method. To be implemented by every child.
   * \param input the input point cloud dataset.
   * \param output the resultant filtered PointCloud2
   */
  virtual void filter(
    const PointCloud2ConstPtr & input, PointCloud2 & output) = 0;

  /** \brief Call the child filter () method, optionally transform the result, and publish it.
   * \param input the input point cloud dataset.
   */
  void computePublish(const PointCloud2ConstPtr & input);

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

private:
  /** \brief PointCloud2 + Indices data callback. */
  void input_indices_callback(const PointCloud2ConstPtr cloud);
};

#endif  // POINTCLOUD_DOWNSAMPLE__FILTERS__FILTER_HPP_
