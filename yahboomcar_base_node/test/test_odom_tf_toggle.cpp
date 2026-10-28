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
#include "tf2_msgs/msg/tf_message.hpp"
#include "yahboomcar_base_node/odom_publisher.hpp"

namespace
{

using namespace std::chrono_literals;

// Topics are remapped under a test-only prefix so a ROS graph running on the
// same machine cannot feed or read this node.
const char kPrefix[] = "/odom_tf_toggle_test";

class OdomTfToggleTest : public ::testing::Test
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
        "-r", "/tf:=" + prefix + "/tf",
        "-r", "/diagnostics:=" + prefix + "/diagnostics",
        "-r", "__ns:=" + prefix});
    odom_node_ = std::make_shared<OdomPublisher>(options);

    helper_ = std::make_shared<rclcpp::Node>("odom_tf_toggle_test_helper");
    joint_pub_ = helper_->create_publisher<sensor_msgs::msg::JointState>(
      prefix + "/joint_states", 50);
    odom_sub_ = helper_->create_subscription<nav_msgs::msg::Odometry>(
      prefix + "/odom", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {last_odom_ = msg;});
    tf_sub_ = helper_->create_subscription<tf2_msgs::msg::TFMessage>(
      prefix + "/tf", 100,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr) {++tf_messages_;});

    executor_.add_node(odom_node_);
    executor_.add_node(helper_);
    spin_until(
      [this]() {
        return joint_pub_->get_subscription_count() > 0 &&
        odom_sub_->get_publisher_count() > 0 &&
        tf_sub_->get_publisher_count() > 0;
      });
  }

  void TearDown() override
  {
    executor_.remove_node(helper_);
    executor_.remove_node(odom_node_);
  }

  // Publishes one wheel state and spins until /odom reflects it, then a little
  // longer so a transform published with it has time to arrive too.
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
    const auto settle = std::chrono::steady_clock::now() + 100ms;
    while (std::chrono::steady_clock::now() < settle) {
      executor_.spin_some(10ms);
    }
  }

  // Drives forward in a few timed steps and returns how many /tf messages
  // arrived meanwhile.
  int drive_and_count_tf(double & angle)
  {
    tf_messages_ = 0;
    for (int i = 0; i < 5; ++i) {
      std::this_thread::sleep_for(20ms);
      angle += 0.5;
      publish_wheels(angle);
    }
    return tf_messages_;
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
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  int tf_messages_ = 0;
};

TEST_F(OdomTfToggleTest, SwitchesTheTransformOffAndBackOnAtRuntime)
{
  double angle = 0.0;
  publish_wheels(angle);
  EXPECT_GT(drive_and_count_tf(angle), 0);

  ASSERT_TRUE(odom_node_->set_parameter(rclcpp::Parameter("pub_odom_tf", false)).successful);
  EXPECT_EQ(drive_and_count_tf(angle), 0);
  // /odom keeps flowing and integrating while the transform is off.
  EXPECT_GT(last_odom_->pose.pose.position.x, 0.1);

  ASSERT_TRUE(odom_node_->set_parameter(rclcpp::Parameter("pub_odom_tf", true)).successful);
  EXPECT_GT(drive_and_count_tf(angle), 0);
}

TEST_F(OdomTfToggleTest, RejectsANonBooleanValue)
{
  const auto result = odom_node_->set_parameter(rclcpp::Parameter("pub_odom_tf", 0));
  EXPECT_FALSE(result.successful);
  double angle = 0.0;
  publish_wheels(angle);
  EXPECT_GT(drive_and_count_tf(angle), 0);
}

}  // namespace
