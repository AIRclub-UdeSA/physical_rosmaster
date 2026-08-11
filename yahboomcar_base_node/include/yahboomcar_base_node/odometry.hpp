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

inline OdomState integrate_velocity(
  const OdomState & state,
  const BodyVelocity & velocity,
  const double dt)
{
  const double delta_heading = velocity.angular_z * dt;
  const double delta_x =
    (velocity.linear_x * std::cos(state.heading) -
    velocity.linear_y * std::sin(state.heading)) * dt;
  const double delta_y =
    (velocity.linear_x * std::sin(state.heading) +
    velocity.linear_y * std::cos(state.heading)) * dt;

  return OdomState{
    state.x + delta_x,
    state.y + delta_y,
    state.heading + delta_heading};
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
  const BodyVelocity & velocity)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_footprint_frame;

  odom.pose.pose.position.x = state.x;
  odom.pose.pose.position.y = state.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = yaw_to_quaternion(state.heading);

  odom.pose.covariance[0] = 0.001;
  odom.pose.covariance[7] = 0.001;
  odom.pose.covariance[35] = 0.001;

  odom.twist.twist.linear.x = velocity.linear_x;
  odom.twist.twist.linear.y = velocity.linear_y;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = velocity.angular_z;

  odom.twist.covariance[0] = 0.0001;
  odom.twist.covariance[7] = 0.0001;
  odom.twist.covariance[35] = 0.0001;

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
