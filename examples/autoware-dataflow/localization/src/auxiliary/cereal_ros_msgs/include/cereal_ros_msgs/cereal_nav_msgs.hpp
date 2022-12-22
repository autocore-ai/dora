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

///use cereal(https://github.com/USCiLab/cereal.git) to serialize the std_msgs in ros

#ifndef CEREAL_ROS_MSGS__CEREAL_NAV_MSGS_HPP_
#define CEREAL_ROS_MSGS__CEREAL_NAV_MSGS_HPP_

#include "cereal/types/string.hpp"
#include "cereal/types/vector.hpp"

#include "cereal_ros_msgs/cereal_std_msgs.hpp"
#include "cereal_ros_msgs/cereal_geometry_msgs.hpp"

#include <nav_msgs/msg/odometry.hpp>

namespace nav_msgs::msg {

template <class Archive>
void serialize(Archive & arch, nav_msgs::msg::Odometry& odometry)
{
    arch(odometry.header, odometry.child_frame_id, odometry.pose, odometry.twist);
}

}

#endif // CEREAL_ROS_MSGS__CEREAL_NAV_MSGS_HPP_
