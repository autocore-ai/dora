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

///use cereal(https://github.com/USCiLab/cereal.git) to serialize the diagnostic_msgs in ros

#ifndef CEREAL_ROS_MSGS__CEREAL_DIAGNOSTIC_MSGS_HPP_
#define CEREAL_ROS_MSGS__CEREAL_DIAGNOSTIC_MSGS_HPP_

#include "cereal/types/string.hpp"
#include "cereal/types/vector.hpp"

#include "cereal_ros_msgs/cereal_std_msgs.hpp"

#include <diagnostic_msgs/msg/key_value.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace diagnostic_msgs::msg {

    template <class Archive>
    void serialize(Archive & arch, diagnostic_msgs::msg::KeyValue& kv)
    {
        arch(kv.key, kv.value);
    }

    template <class Archive>
    void serialize(Archive & arch, diagnostic_msgs::msg::DiagnosticStatus& status)
    {
        arch(status.level, status.name, status.message, status.hardware_id, status.values);
    }

    template <class Archive>
    void serialize(Archive & arch, diagnostic_msgs::msg::DiagnosticArray& array)
    {
        arch(array.header, array.status);
    }

}

#endif // CEREAL_ROS_MSGS__CEREAL_DIAGNOSTIC_MSGS_HPP_
