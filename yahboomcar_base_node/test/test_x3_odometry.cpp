#include <gtest/gtest.h>

#include <cmath>

#include "yahboomcar_base_node/odometry.hpp"

namespace
{

TEST(X3Odometry, PreservesLateralMecanumTwist)
{
  yahboomcar_base_node::OdomState state{1.0, 2.0, 0.25};
  yahboomcar_base_node::BodyVelocity velocity{0.4, -0.3, 0.7};

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 345;

  const auto odom = yahboomcar_base_node::make_odometry_msg(
    stamp, "odom_test", "base_footprint_test", state, velocity);

  EXPECT_EQ(odom.header.frame_id, "odom_test");
  EXPECT_EQ(odom.child_frame_id, "base_footprint_test");
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.x, velocity.linear_x);
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.y, velocity.linear_y);
  EXPECT_DOUBLE_EQ(odom.twist.twist.angular.z, velocity.angular_z);
}

TEST(X3Odometry, IntegratesLateralVelocityAtZeroHeading)
{
  const yahboomcar_base_node::OdomState state{0.0, 0.0, 0.0};
  const yahboomcar_base_node::BodyVelocity velocity{0.0, 0.5, 0.0};

  const auto result = yahboomcar_base_node::integrate_velocity(state, velocity, 2.0);

  EXPECT_DOUBLE_EQ(result.x, 0.0);
  EXPECT_DOUBLE_EQ(result.y, 1.0);
  EXPECT_DOUBLE_EQ(result.heading, 0.0);
}

TEST(X3Odometry, RotatesBodyVelocityIntoOdomFrame)
{
  const double half_pi = std::acos(-1.0) / 2.0;
  const yahboomcar_base_node::OdomState state{0.0, 0.0, half_pi};
  const yahboomcar_base_node::BodyVelocity velocity{1.0, 0.0, 0.0};

  const auto result = yahboomcar_base_node::integrate_velocity(state, velocity, 1.0);

  EXPECT_NEAR(result.x, 0.0, 1e-12);
  EXPECT_NEAR(result.y, 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(result.heading, half_pi);
}

TEST(X3Odometry, MecanumKinematicsForward)
{
  yahboomcar_base_node::MecanumParams params{0.033, 0.10, 0.085};
  // 4 wheels move forward 10 rad in 1 sec => w = 10 rad/s
  // vx = r * w = 0.033 * 10 = 0.33 m/s
  yahboomcar_base_node::WheelDisplacements deltas{10.0, 10.0, 10.0, 10.0};

  const auto vel = yahboomcar_base_node::compute_mecanum_body_velocity(deltas, params, 1.0);

  EXPECT_NEAR(vel.linear_x, 0.33, 1e-6);
  EXPECT_NEAR(vel.linear_y, 0.0, 1e-6);
  EXPECT_NEAR(vel.angular_z, 0.0, 1e-6);
}

TEST(X3Odometry, MecanumKinematicsStrafeLeft)
{
  yahboomcar_base_node::MecanumParams params{0.033, 0.10, 0.085};
  // FL: -10, FR: +10, BL: +10, BR: -10 => vy = r * w = 0.33 m/s
  yahboomcar_base_node::WheelDisplacements deltas{-10.0, 10.0, 10.0, -10.0};

  const auto vel = yahboomcar_base_node::compute_mecanum_body_velocity(deltas, params, 1.0);

  EXPECT_NEAR(vel.linear_x, 0.0, 1e-6);
  EXPECT_NEAR(vel.linear_y, 0.33, 1e-6);
  EXPECT_NEAR(vel.angular_z, 0.0, 1e-6);
}

TEST(X3Odometry, MecanumKinematicsRotateCCW)
{
  yahboomcar_base_node::MecanumParams params{0.033, 0.10, 0.085}; // lx+ly = 0.185
  // FL: -10, FR: +10, BL: -10, BR: +10
  // wz = (r / (4 * (lx+ly))) * (10 + 10 + 10 + 10) = 0.033 * 40 / (4 * 0.185) = 1.32 / 0.74 = 1.78378 rad/s
  yahboomcar_base_node::WheelDisplacements deltas{-10.0, 10.0, -10.0, 10.0};

  const auto vel = yahboomcar_base_node::compute_mecanum_body_velocity(deltas, params, 1.0);

  EXPECT_NEAR(vel.linear_x, 0.0, 1e-6);
  EXPECT_NEAR(vel.linear_y, 0.0, 1e-6);
  EXPECT_NEAR(vel.angular_z, 0.033 * 40.0 / (4.0 * 0.185), 1e-6);
}

}  // namespace
