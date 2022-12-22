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

///use cereal(https://github.com/USCiLab/cereal.git) to serialize the geometry_msgs in ros

#ifndef CEREAL_ROS_MSGS__CEREAL_GEOMETRY_MSGS_HPP_
#define CEREAL_ROS_MSGS__CEREAL_GEOMETRY_MSGS_HPP_

#include "cereal/types/string.hpp"
#include "cereal/types/vector.hpp"
#include "cereal/types/array.hpp"

#include "cereal_ros_msgs/cereal_std_msgs.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>


namespace geometry_msgs::msg {

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Point& point)
{
    arch(point.x, point.y, point.z);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Quaternion& quaternion)
{
    arch(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::QuaternionStamped& qua_stamped)
{
    arch(qua_stamped.header, qua_stamped.quaternion);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Pose& pose)
{
    arch(pose.position, pose.orientation);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::PoseStamped& pose_stamped)
{
    arch(pose_stamped.header, pose_stamped.pose);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::PoseWithCovariance& pose_cov)
{
    arch(pose_cov.pose, pose_cov.covariance);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::PoseWithCovarianceStamped& pose_cov_stamped)
{
    arch(pose_cov_stamped.header, pose_cov_stamped.pose);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Vector3& vector3)
{
    arch(vector3.x, vector3.y, vector3.z);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Transform& trans)
{
    arch(trans.translation, trans.rotation);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::TransformStamped& trans_stamped)
{
    arch(trans_stamped.header, trans_stamped.child_frame_id, trans_stamped.transform);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Twist& twist)
{
    arch(twist.linear, twist.angular);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::TwistStamped& twist_stamped)
{
    arch(twist_stamped.header, twist_stamped.twist);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::TwistWithCovariance& twist_cov)
{
    arch(twist_cov.twist, twist_cov.covariance);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::TwistWithCovarianceStamped& twist_cov_stamped)
{
    arch(twist_cov_stamped.header, twist_cov_stamped.twist);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Point32& point32)
{
    arch(point32.x, point32.y, point32.z);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::Polygon& polygon)
{
    arch(polygon.points);
}

template <class Archive>
void serialize(Archive & arch, geometry_msgs::msg::PolygonStamped& polygon_stamped)
{
    arch(polygon_stamped.header, polygon_stamped.polygon);
}

}

#endif // CEREAL_ROS_MSGS__CEREAL_GEOMETRY_MSGS_HPP_
