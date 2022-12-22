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

#ifndef CEREAL_ROS_MSGS__CEREAL_TIER4_DEBUG_MSGS_HPP_
#define CEREAL_ROS_MSGS__CEREAL_TIER4_DEBUG_MSGS_HPP_

#include "cereal/types/string.hpp"
#include "cereal/types/vector.hpp"

#include "cereal_ros_msgs/cereal_std_msgs.hpp"

#include <tier4_debug_msgs/msg/bool_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <tier4_debug_msgs/msg/int64_stamped.hpp>
#include <tier4_debug_msgs/msg/multi_array_dimension.hpp>
#include <tier4_debug_msgs/msg/multi_array_layout.hpp>
#include <tier4_debug_msgs/msg/int32_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/int64_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/string_stamped.hpp>


namespace tier4_debug_msgs::msg {

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::BoolStamped& bs)
{
    arch(bs.stamp, bs.data);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::Int32Stamped& i32_stamped)
{
    arch(i32_stamped.stamp, i32_stamped.data);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::Int64Stamped& i64_stamped)
{
    arch(i64_stamped.stamp, i64_stamped.data);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::Float32Stamped& f32_stamped)
{
    arch(f32_stamped.stamp, f32_stamped.data);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::Float64Stamped& f64_stamped)
{
    arch(f64_stamped.stamp, f64_stamped.data);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::MultiArrayDimension& ma_dim)
{
    arch(ma_dim.label, ma_dim.size, ma_dim.stride);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::MultiArrayLayout& ma_layout)
{
    arch(ma_layout.dim, ma_layout.data_offset);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::Int32MultiArrayStamped& i32ma_stamped)
{
    arch(i32ma_stamped.stamp, i32ma_stamped.layout, i32ma_stamped.data);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::Int64MultiArrayStamped& i64ma_stamped)
{
    arch(i64ma_stamped.stamp, i64ma_stamped.layout, i64ma_stamped.data);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::Float32MultiArrayStamped& f32ma_stamped)
{
    arch(f32ma_stamped.stamp, f32ma_stamped.layout, f32ma_stamped.data);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::Float64MultiArrayStamped& f64ma_stamped)
{
    arch(f64ma_stamped.stamp, f64ma_stamped.layout, f64ma_stamped.data);
}

template <class Archive>
void serialize(Archive & arch, tier4_debug_msgs::msg::StringStamped& str_stamped)
{
    arch(str_stamped.stamp, str_stamped.data);
}

}

#endif // CEREAL_ROS_MSGS__CEREAL_TIER4_DEBUG_MSGS_HPP_
