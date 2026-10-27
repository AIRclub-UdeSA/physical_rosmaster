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

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yahboomcar_base_node/odom_publisher.hpp"

namespace
{

using namespace std::chrono_literals;

// Topics are remapped under a test-only prefix so a ROS graph running on the
// same machine cannot feed or read this node.
const char kPrefix[] = "/reset_odom_test";

class ResetOdomTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    const std::string prefix(kPrefix);
    rclcpp::NodeOptions options;
    options.arguments(
      {
        "--ros-args",
        "-r", "/odom:=" + prefix + "/odom",
        "-r", "/diagnostics:=" + prefix + "/diagnostics",
        "-r", "__ns:=" + prefix,
        "-p", "pub_odom_tf:=false"});
    odom_node_ = std::make_shared<OdomPublisher>(options);

    helper_ = std::make_shared<rclcpp::Node>("reset_odom_test_helper");
    joint_pub_ = helper_->create_publisher<sensor_msgs::msg::JointState>(
      prefix + "/joint_states", 50);
    odom_sub_ = helper_->create_subscription<nav_msgs::msg::Odometry>(
      prefix + "/odom", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {last_odom_ = msg;});
    reset_client_ = helper_->create_client<std_srvs::srv::Trigger>(
      prefix + "/base_node/reset_odom");

    executor_.add_node(odom_node_);
    executor_.add_node(helper_);
    ASSERT_TRUE(reset_client_->wait_for_service(5s));
    spin_until(
      [this]() {
        return joint_pub_->get_subscription_count() > 0 &&
        odom_sub_->get_publisher_count() > 0;
      });
  }

  void TearDown() override
  {
    executor_.remove_node(helper_);
    executor_.remove_node(odom_node_);
  }

  // Publishes all four wheels at the same angle, which drives the robot
  // straight forward, and spins until /odom reflects that message.
  void publish_wheels(const double angle)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = helper_->get_clock()->now();
    msg.name = {"front_left_wheel_joint", "front_right_wheel_joint",
      "back_left_wheel_joint", "back_right_wheel_joint"};
    msg.position = {angle, angle, angle, angle};
    last_odom_.reset();
    joint_pub_->publish(msg);
    spin_until([this]() {return last_odom_ != nullptr;});
    ASSERT_NE(last_odom_, nullptr);
  }

  // Drives the wheels forward in small, timed steps from angle.
  double drive_forward(double angle, const int steps)
  {
    for (int i = 0; i < steps; ++i) {
      std::this_thread::sleep_for(20ms);
      angle += 0.5;
      publish_wheels(angle);
    }
    return angle;
  }

  std_srvs::srv::Trigger::Response::SharedPtr call_reset()
  {
    auto future = reset_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto result = executor_.spin_until_future_complete(future, 5s);
    EXPECT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    return result == rclcpp::FutureReturnCode::SUCCESS ? future.get() : nullptr;
  }

  void spin_until(const std::function<bool()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!done() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(10ms);
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<OdomPublisher> odom_node_;
  rclcpp::Node::SharedPtr helper_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
};

TEST_F(ResetOdomTest, ZeroesPoseAndKeepsIntegratingFromThere)
{
  double angle = 0.0;
  publish_wheels(angle);
  angle = drive_forward(angle, 10);
  const double distance_before_reset = last_odom_->pose.pose.position.x;
  ASSERT_GT(distance_before_reset, 0.1);

  const auto response = call_reset();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "Odometry pose reset to zero");

  // Same wheel angle as before the reset: the robot has not moved since, so
  // the next /odom is exactly the origin.
  std::this_thread::sleep_for(20ms);
  publish_wheels(angle);
  EXPECT_DOUBLE_EQ(last_odom_->pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(last_odom_->pose.pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(last_odom_->pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(last_odom_->pose.pose.orientation.w, 1.0);

  // The encoder baseline was kept, so driving again integrates from zero
  // instead of jumping by the wheel travel accumulated before the reset.
  drive_forward(angle, 10);
  EXPECT_GT(last_odom_->pose.pose.position.x, 0.0);
  EXPECT_NEAR(last_odom_->pose.pose.position.x, distance_before_reset, 1e-6);
}

TEST_F(ResetOdomTest, ReportsWhenEncodersHaveNotStarted)
{
  const auto response = call_reset();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(
    response->message,
    "Odometry pose reset to zero; /odom resumes when wheel encoders do");
}

}  // namespace
