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

#ifndef CEREAL_ROS_MSGS__CEREAL_STD_MSGS_HPP_
#define CEREAL_ROS_MSGS__CEREAL_STD_MSGS_HPP_

#include "cereal/types/string.hpp"
#include "cereal/access.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

namespace builtin_interfaces::msg {
    template <class Archive>
    void serialize(Archive & arch, builtin_interfaces::msg::Time& tm)
    {
        arch(tm.sec, tm.nanosec);
    }
}

namespace std_msgs::msg {

template <class Archive>
void serialize(Archive & arch, std_msgs::msg::Header& header)
{
    arch(header.stamp, header.frame_id);
}

template <class Archive>
void serialize(Archive & arch, std_msgs::msg::Float32& flt)
{
    arch(flt.data);
}

template <class Archive>
void serialize(Archive & arch, std_msgs::msg::String& str)
{
    arch(str.data);
}
// template<class Archive>
// static void load_and_construct(Archive & ar, cereal::construct<std_msgs::msg::String> & construct)
// {
//     std::string data;
//     ar(data);
//     construct(data);
// }

}

#endif // CEREAL_ROS_MSGS__CEREAL_STD_MSGS_HPP_
