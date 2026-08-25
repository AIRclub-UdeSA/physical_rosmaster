// Copyright 2026 AIRclub UdeSA
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef YAHBOOMCAR_BASE_NODE__ODOMETRY_HPP_
#define YAHBOOMCAR_BASE_NODE__ODOMETRY_HPP_

#include <cmath>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace yahboomcar_base_node
{

struct OdomState
{
  double x;
  double y;
  double heading;
};

struct BodyVelocity
{
  double linear_x;
  double linear_y;
  double angular_z;
};

struct WheelDisplacements
{
  double delta_fl;
  double delta_fr;
  double delta_bl;
  double delta_br;
};

struct MecanumParams
{
  double wheel_radius;  // m (e.g. 0.033)
  double lx;            // m (half wheelbase x separation)
  double ly;            // m (half track width y separation)
};

struct OdomCovariances
{
  double pose_x;
  double pose_y;
  double pose_yaw;
  double twist_x;
  double twist_y;
  double twist_yaw;
};

inline BodyVelocity compute_mecanum_body_velocity(
  const WheelDisplacements & wheel_deltas,
  const MecanumParams & params,
  const double dt)
{
  if (dt <= 0.0) {
    return BodyVelocity{0.0, 0.0, 0.0};
  }

  const double r = params.wheel_radius;
  const double k = (params.lx + params.ly > 0.0) ? (params.lx + params.ly) : 1.0;

  const double vx = (r / 4.0) *
    (wheel_deltas.delta_fl + wheel_deltas.delta_fr + wheel_deltas.delta_bl +
    wheel_deltas.delta_br) / dt;
  const double vy = (r / 4.0) *
    (-wheel_deltas.delta_fl + wheel_deltas.delta_fr + wheel_deltas.delta_bl -
    wheel_deltas.delta_br) / dt;
  const double wz = (r / (4.0 * k)) *
    (-wheel_deltas.delta_fl + wheel_deltas.delta_fr - wheel_deltas.delta_bl +
    wheel_deltas.delta_br) / dt;

  return BodyVelocity{vx, vy, wz};
}

inline BodyVelocity scale_body_velocity(
  const BodyVelocity & velocity,
  const double linear_scale_x,
  const double linear_scale_y,
  const double angular_scale)
{
  return BodyVelocity{
    velocity.linear_x * linear_scale_x,
    velocity.linear_y * linear_scale_y,
    velocity.angular_z * angular_scale};
}

inline bool is_source_fresh(const double age, const double timeout)
{
  return std::isfinite(age) && std::isfinite(timeout) &&
         age >= 0.0 && timeout > 0.0 && age <= timeout;
}

inline OdomState integrate_velocity(
  const OdomState & state,
  const BodyVelocity & velocity,
  const double dt)
{
  const double delta_heading = velocity.angular_z * dt;
  const double midpoint_heading = state.heading + (delta_heading / 2.0);
  const double delta_x =
    (velocity.linear_x * std::cos(midpoint_heading) -
    velocity.linear_y * std::sin(midpoint_heading)) * dt;
  const double delta_y =
    (velocity.linear_x * std::sin(midpoint_heading) +
    velocity.linear_y * std::cos(midpoint_heading)) * dt;

  return OdomState{
    state.x + delta_x,
    state.y + delta_y,
    std::atan2(
      std::sin(state.heading + delta_heading),
      std::cos(state.heading + delta_heading))};
}

inline geometry_msgs::msg::Quaternion yaw_to_quaternion(const double heading)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, heading);

  geometry_msgs::msg::Quaternion msg;
  msg.x = quaternion.x();
  msg.y = quaternion.y();
  msg.z = quaternion.z();
  msg.w = quaternion.w();
  return msg;
}

inline nav_msgs::msg::Odometry make_odometry_msg(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & odom_frame,
  const std::string & base_footprint_frame,
  const OdomState & state,
  const BodyVelocity & velocity,
  const OdomCovariances & covariances =
  OdomCovariances{0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001})
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_footprint_frame;

  odom.pose.pose.position.x = state.x;
  odom.pose.pose.position.y = state.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = yaw_to_quaternion(state.heading);

  odom.pose.covariance[0] = covariances.pose_x;
  odom.pose.covariance[7] = covariances.pose_y;
  odom.pose.covariance[35] = covariances.pose_yaw;

  odom.twist.twist.linear.x = velocity.linear_x;
  odom.twist.twist.linear.y = velocity.linear_y;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = velocity.angular_z;

  odom.twist.covariance[0] = covariances.twist_x;
  odom.twist.covariance[7] = covariances.twist_y;
  odom.twist.covariance[35] = covariances.twist_yaw;

  return odom;
}

inline geometry_msgs::msg::TransformStamped make_odom_transform(
  const builtin_interfaces::msg::Time & stamp,
  const std::string & odom_frame,
  const std::string & base_footprint_frame,
  const OdomState & state)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = odom_frame;
  transform.child_frame_id = base_footprint_frame;
  transform.transform.translation.x = state.x;
  transform.transform.translation.y = state.y;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = yaw_to_quaternion(state.heading);
  return transform;
}

}  // namespace yahboomcar_base_node

#endif  // YAHBOOMCAR_BASE_NODE__ODOMETRY_HPP_
