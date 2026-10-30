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
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
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
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        ++tf_messages_;
        last_tf_ = msg;
      });
    diagnostics_sub_ = helper_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      prefix + "/diagnostics", 10,
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        for (const auto & status : msg->status) {
          for (const auto & value : status.values) {
            if (value.key == "odom_tf") {
              last_odom_tf_status_ = value.value;
            }
          }
        }
      });

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

  // Spins long enough for anything already in flight (a transform published
  // before a toggle, a diagnostic published by it) to be delivered.
  void settle()
  {
    const auto until = std::chrono::steady_clock::now() + 200ms;
    while (std::chrono::steady_clock::now() < until) {
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
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  tf2_msgs::msg::TFMessage::SharedPtr last_tf_;
  std::string last_odom_tf_status_;
  int tf_messages_ = 0;
};

TEST_F(OdomTfToggleTest, SwitchesTheTransformOffAndBackOnAtRuntime)
{
  double angle = 0.0;
  publish_wheels(angle);
  EXPECT_GT(drive_and_count_tf(angle), 0);

  ASSERT_TRUE(odom_node_->set_parameter(rclcpp::Parameter("pub_odom_tf", false)).successful);
  settle();
  const double x_before_off = last_odom_->pose.pose.position.x;
  EXPECT_EQ(drive_and_count_tf(angle), 0);
  // /odom keeps flowing and integrating while the transform is off: five
  // steps of 0.5 rad on a 0.0325 m wheel are about 0.08 m.
  EXPECT_GT(last_odom_->pose.pose.position.x, x_before_off + 0.05);

  ASSERT_TRUE(odom_node_->set_parameter(rclcpp::Parameter("pub_odom_tf", true)).successful);
  EXPECT_GT(drive_and_count_tf(angle), 0);
  // The restored transform carries the pose that kept integrating meanwhile.
  ASSERT_NE(last_tf_, nullptr);
  ASSERT_EQ(last_tf_->transforms.size(), 1u);
  const auto & transform = last_tf_->transforms.front();
  EXPECT_EQ(transform.header.frame_id, "odom");
  EXPECT_EQ(transform.child_frame_id, "base_footprint");
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, last_odom_->pose.pose.position.x);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, last_odom_->pose.pose.position.y);
}

TEST_F(OdomTfToggleTest, ReportsTheStateOnDiagnostics)
{
  // Before any toggle the status comes from the 1 s diagnostic timer.
  spin_until([this]() {return !last_odom_tf_status_.empty();});
  EXPECT_EQ(last_odom_tf_status_, "published");

  ASSERT_TRUE(odom_node_->set_parameter(rclcpp::Parameter("pub_odom_tf", false)).successful);
  settle();
  EXPECT_EQ(last_odom_tf_status_, "disabled by pub_odom_tf parameter");

  ASSERT_TRUE(odom_node_->set_parameter(rclcpp::Parameter("pub_odom_tf", true)).successful);
  settle();
  EXPECT_EQ(last_odom_tf_status_, "published");
}

// ros2 param set reaches the callback through the node's parameter service,
// not through set_parameter() on the node object.
TEST_F(OdomTfToggleTest, SwitchesThroughTheParameterService)
{
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(
    helper_, odom_node_->get_fully_qualified_name());
  spin_until([&client]() {return client->service_is_ready();});
  ASSERT_TRUE(client->service_is_ready());

  auto future = client->set_parameters({rclcpp::Parameter("pub_odom_tf", false)});
  spin_until(
    [&future]() {return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;});
  ASSERT_EQ(future.wait_for(std::chrono::seconds(0)), std::future_status::ready);
  const std::vector<rcl_interfaces::msg::SetParametersResult> results = future.get();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_TRUE(results.front().successful);

  double angle = 0.0;
  publish_wheels(angle);
  settle();
  EXPECT_EQ(drive_and_count_tf(angle), 0);
}

// rclcpp rejects a value of the wrong type against the declared bool before
// the node's callback runs, so this checks the declaration, not the callback.
TEST_F(OdomTfToggleTest, DeclaredTypeRejectsANonBooleanValue)
{
  const auto result = odom_node_->set_parameter(rclcpp::Parameter("pub_odom_tf", 0));
  EXPECT_FALSE(result.successful);
  double angle = 0.0;
  publish_wheels(angle);
  EXPECT_GT(drive_and_count_tf(angle), 0);
}

// The callback sees every parameter set on the node; one it does not handle
// must still succeed and leave the transform alone.
TEST_F(OdomTfToggleTest, OtherParametersStillSucceed)
{
  EXPECT_TRUE(
    odom_node_->set_parameter(rclcpp::Parameter("joint_state_timeout", 0.5)).successful);
  double angle = 0.0;
  publish_wheels(angle);
  EXPECT_GT(drive_and_count_tf(angle), 0);
}

}  // namespace
