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

///use cereal(https://github.com/USCiLab/cereal.git) to serialize the sensor_msgs in ros

#ifndef CEREAL_ROS_MSGS__CEREAL_SENSOR_MSGS_HPP_
#define CEREAL_ROS_MSGS__CEREAL_SENSOR_MSGS_HPP_

#include "cereal/types/string.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/array.hpp"

#include "cereal_ros_msgs/cereal_std_msgs.hpp"
#include "cereal_ros_msgs/cereal_geometry_msgs.hpp"

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace sensor_msgs::msg {

template <class Archive>
void serialize(Archive & arch, sensor_msgs::msg::PointField& field)
{
    arch(field.name, field.offset, field.datatype, field.count);
}

template <class Archive>
void serialize(Archive & arch, sensor_msgs::msg::PointCloud2& cloud2)
{
    arch(cloud2.header, cloud2.height, cloud2.width, cloud2.fields, cloud2.is_bigendian, cloud2.point_step, cloud2.row_step, cloud2.data, cloud2.is_dense);
}

template <class Archive>
void serialize(Archive & arch, sensor_msgs::msg::Imu& imu)
{
    arch(imu.header, imu.orientation, imu.orientation_covariance, imu.angular_velocity, 
        imu.angular_velocity_covariance, imu.linear_acceleration, imu.linear_acceleration_covariance);
}

template <class Archive>
void serialize(Archive & arch, sensor_msgs::msg::NavSatStatus& status)
{
    arch(status.status, status.service);
}

template <class Archive>
void serialize(Archive & arch, sensor_msgs::msg::NavSatFix& fix)
{
    arch(fix.header, fix.status, fix.latitude, fix.longitude, fix.altitude,
        fix.position_covariance, fix.position_covariance_type);
}

}

#endif // CEREAL_ROS_MSGS__CEREAL_SENSOR_MSGS_HPP_
