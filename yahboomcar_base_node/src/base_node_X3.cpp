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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "yahboomcar_base_node/odometry.hpp"

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
  : Node("base_node")
  {
    this->declare_parameter<double>("wheel_radius", 0.0325);
    this->declare_parameter<double>("wheelbase_x", 0.160);
    this->declare_parameter<double>("wheelbase_y", 0.170);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
    this->declare_parameter<double>("linear_scale_x", 1.0);
    this->declare_parameter<double>("linear_scale_y", 1.0);
    this->declare_parameter<double>("angular_scale", 1.0);
    this->declare_parameter<bool>("pub_odom_tf", true);
    this->declare_parameter<double>("joint_state_timeout", 0.5);
    this->declare_parameter<double>("max_wheel_delta_rad", 20.0);
    this->declare_parameter<std::vector<std::string>>(
      "wheel_joint_names",
      {"front_left_wheel_joint", "front_right_wheel_joint",
        "back_left_wheel_joint", "back_right_wheel_joint"});
    this->declare_parameter<double>("pose_covariance_x", 0.001);
    this->declare_parameter<double>("pose_covariance_y", 0.001);
    this->declare_parameter<double>("pose_covariance_yaw", 0.001);
    this->declare_parameter<double>("twist_covariance_x", 0.0001);
    this->declare_parameter<double>("twist_covariance_y", 0.0001);
    this->declare_parameter<double>("twist_covariance_yaw", 0.0001);

    this->get_parameter<double>("linear_scale_x", linear_scale_x_);
    this->get_parameter<double>("linear_scale_y", linear_scale_y_);
    this->get_parameter<double>("angular_scale", angular_scale_);
    this->get_parameter<double>("wheel_radius", mecanum_params_.wheel_radius);
    double wx = 0.160, wy = 0.170;
    this->get_parameter<double>("wheelbase_x", wx);
    this->get_parameter<double>("wheelbase_y", wy);
    mecanum_params_.lx = wx / 2.0;
    mecanum_params_.ly = wy / 2.0;

    this->get_parameter<bool>("pub_odom_tf", pub_odom_tf_);
    this->get_parameter<std::string>("odom_frame", odom_frame_);
    this->get_parameter<std::string>("base_footprint_frame", base_footprint_frame_);
    this->get_parameter<double>("joint_state_timeout", joint_state_timeout_);
    this->get_parameter<double>("max_wheel_delta_rad", max_wheel_delta_rad_);
    this->get_parameter<std::vector<std::string>>("wheel_joint_names", wheel_joint_names_);
    this->get_parameter<double>("pose_covariance_x", covariances_.pose_x);
    this->get_parameter<double>("pose_covariance_y", covariances_.pose_y);
    this->get_parameter<double>("pose_covariance_yaw", covariances_.pose_yaw);
    this->get_parameter<double>("twist_covariance_x", covariances_.twist_x);
    this->get_parameter<double>("twist_covariance_y", covariances_.twist_y);
    this->get_parameter<double>("twist_covariance_yaw", covariances_.twist_yaw);

    if (wheel_joint_names_.size() != 4 ||
      !std::isfinite(mecanum_params_.wheel_radius) || mecanum_params_.wheel_radius <= 0.0 ||
      !std::isfinite(mecanum_params_.lx) || mecanum_params_.lx <= 0.0 ||
      !std::isfinite(mecanum_params_.ly) || mecanum_params_.ly <= 0.0 ||
      !std::isfinite(joint_state_timeout_) || joint_state_timeout_ <= 0.0 ||
      !std::isfinite(max_wheel_delta_rad_) || max_wheel_delta_rad_ <= 0.0)
    {
      throw std::invalid_argument("invalid X3 wheel odometry configuration");
    }

    const rclcpp::Time now = this->get_clock()->now();
    last_joint_time_ = now;
    last_joint_receive_time_ = now;
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      50,
      std::bind(&OdomPublisher::handle_joint_states, this, std::placeholders::_1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    diagnostic_publisher_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    sensor_watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&OdomPublisher::handle_sensor_timeout, this));
    diagnostic_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&OdomPublisher::publish_encoder_diagnostic, this));
    set_encoder_health(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Waiting for canonical wheel encoder joint states");
  }

private:
  void publish_encoder_diagnostic()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = this->get_clock()->now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = encoder_health_level_;
    status.name = "yahboomcar_base_node: wheel encoder odometry";
    status.message = encoder_health_message_;
    status.hardware_id = "rosmaster_x3_wheel_encoders";

    diagnostic_msgs::msg::KeyValue source;
    source.key = "source";
    source.value = "joint_states encoder positions";
    status.values.push_back(source);

    diagnostic_msgs::msg::KeyValue timeout;
    timeout.key = "timeout_seconds";
    timeout.value = std::to_string(joint_state_timeout_);
    status.values.push_back(timeout);

    array.status.push_back(status);
    diagnostic_publisher_->publish(array);
  }

  void set_encoder_health(const uint8_t level, const std::string & message)
  {
    const bool changed = level != encoder_health_level_ || message != encoder_health_message_;
    encoder_health_level_ = level;
    encoder_health_message_ = message;
    if (changed && diagnostic_publisher_) {
      publish_encoder_diagnostic();
    }
  }

  void publish_odom_and_tf(
    const rclcpp::Time & stamp,
    const yahboomcar_base_node::BodyVelocity & velocity)
  {
    const yahboomcar_base_node::OdomState state{x_pos_, y_pos_, heading_};
    const builtin_interfaces::msg::Time msg_stamp = stamp;

    const nav_msgs::msg::Odometry odom =
      yahboomcar_base_node::make_odometry_msg(
      msg_stamp,
      odom_frame_,
      base_footprint_frame_,
      state,
      velocity,
      covariances_);
    odom_publisher_->publish(odom);

    if (pub_odom_tf_) {
      const geometry_msgs::msg::TransformStamped transform =
        yahboomcar_base_node::make_odom_transform(
        msg_stamp,
        odom_frame_,
        base_footprint_frame_,
        state);
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void handle_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.empty() || msg->position.empty()) {
      set_encoder_health(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Ignoring joint_states without wheel names or positions");
      return;
    }

    int idx_fl = -1, idx_fr = -1, idx_bl = -1, idx_br = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto & name = msg->name[i];
      if (name == wheel_joint_names_[0]) {
        idx_fl = i;
      } else if (name == wheel_joint_names_[1]) {
        idx_fr = i;
      } else if (name == wheel_joint_names_[2]) {
        idx_bl = i;
      } else if (name == wheel_joint_names_[3]) {idx_br = i;}
    }

    if (idx_fl < 0 || idx_fr < 0 || idx_bl < 0 || idx_br < 0) {
      set_encoder_health(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Canonical wheel joints are missing from joint_states");
      return;
    }

    const size_t largest_index = static_cast<size_t>(
      std::max(std::max(idx_fl, idx_fr), std::max(idx_bl, idx_br)));
    if (largest_index >= msg->position.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Ignoring malformed joint_states: wheel names exceed position array");
      set_encoder_health(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Wheel names exceed the joint_states position array");
      return;
    }

    const double pos_fl = msg->position[idx_fl];
    const double pos_fr = msg->position[idx_fr];
    const double pos_bl = msg->position[idx_bl];
    const double pos_br = msg->position[idx_br];

    if (!std::isfinite(pos_fl) || !std::isfinite(pos_fr) ||
      !std::isfinite(pos_bl) || !std::isfinite(pos_br))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Ignoring non-finite wheel positions");
      set_encoder_health(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Wheel encoder positions contain non-finite values");
      return;
    }

    const rclcpp::Time arrival_time = this->get_clock()->now();
    const bool stream_was_fresh = joint_data_received_ &&
      yahboomcar_base_node::is_source_fresh(
      (arrival_time - last_joint_receive_time_).seconds(), joint_state_timeout_);
    last_joint_receive_time_ = arrival_time;
    joint_data_received_ = true;

    const rclcpp::Time current_time = msg->header.stamp.sec != 0 ?
      rclcpp::Time(msg->header.stamp) : arrival_time;

    if (!has_prev_joints_ || !stream_was_fresh) {
      prev_pos_fl_ = pos_fl;
      prev_pos_fr_ = pos_fr;
      prev_pos_bl_ = pos_bl;
      prev_pos_br_ = pos_br;
      last_joint_time_ = current_time;
      has_prev_joints_ = true;
      publish_odom_and_tf(
        current_time, yahboomcar_base_node::BodyVelocity{0.0, 0.0, 0.0});
      stale_zero_published_ = false;
      set_encoder_health(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Wheel encoder stream healthy; odometry input rebased");
      return;
    }

    const double dt = (current_time - last_joint_time_).seconds();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > joint_state_timeout_) {
      prev_pos_fl_ = pos_fl;
      prev_pos_fr_ = pos_fr;
      prev_pos_bl_ = pos_bl;
      prev_pos_br_ = pos_br;
      last_joint_time_ = current_time;
      publish_odom_and_tf(
        current_time, yahboomcar_base_node::BodyVelocity{0.0, 0.0, 0.0});
      stale_zero_published_ = false;
      set_encoder_health(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Encoder timestamp discontinuity rejected; input rebased");
      return;
    }
    last_joint_time_ = current_time;
    const yahboomcar_base_node::WheelDisplacements deltas{
      pos_fl - prev_pos_fl_,
      pos_fr - prev_pos_fr_,
      pos_bl - prev_pos_bl_,
      pos_br - prev_pos_br_
    };

    prev_pos_fl_ = pos_fl;
    prev_pos_fr_ = pos_fr;
    prev_pos_bl_ = pos_bl;
    prev_pos_br_ = pos_br;

    if (std::abs(deltas.delta_fl) > max_wheel_delta_rad_ ||
      std::abs(deltas.delta_fr) > max_wheel_delta_rad_ ||
      std::abs(deltas.delta_bl) > max_wheel_delta_rad_ ||
      std::abs(deltas.delta_br) > max_wheel_delta_rad_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected wheel-position discontinuity and rebased odometry input");
      publish_odom_and_tf(current_time, yahboomcar_base_node::BodyVelocity{0.0, 0.0, 0.0});
      stale_zero_published_ = true;
      set_encoder_health(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Wheel-position discontinuity rejected; input rebased");
      return;
    }

    const yahboomcar_base_node::BodyVelocity raw_velocity =
      yahboomcar_base_node::compute_mecanum_body_velocity(deltas, mecanum_params_, dt);
    const yahboomcar_base_node::BodyVelocity velocity =
      yahboomcar_base_node::scale_body_velocity(
      raw_velocity, linear_scale_x_, linear_scale_y_, angular_scale_);

    if (!std::isfinite(velocity.linear_x) || !std::isfinite(velocity.linear_y) ||
      !std::isfinite(velocity.angular_z))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Rejected non-finite velocity calculated from wheel encoders");
      publish_odom_and_tf(
        current_time, yahboomcar_base_node::BodyVelocity{0.0, 0.0, 0.0});
      stale_zero_published_ = true;
      set_encoder_health(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Calculated wheel odometry velocity is non-finite");
      return;
    }

    const yahboomcar_base_node::OdomState state =
      yahboomcar_base_node::integrate_velocity(
      yahboomcar_base_node::OdomState{x_pos_, y_pos_, heading_},
      velocity,
      dt);

    x_pos_ = state.x;
    y_pos_ = state.y;
    heading_ = state.heading;

    publish_odom_and_tf(current_time, velocity);
    stale_zero_published_ = false;
    set_encoder_health(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Wheel encoder odometry healthy");
  }

  void handle_sensor_timeout()
  {
    if (!joint_data_received_) {
      return;
    }

    const rclcpp::Time now = this->get_clock()->now();
    const bool joint_states_fresh =
      yahboomcar_base_node::is_source_fresh(
      (now - last_joint_receive_time_).seconds(), joint_state_timeout_);

    if (!joint_states_fresh && !stale_zero_published_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Wheel encoders timed out; integration stopped (no vel_raw fallback)");
      publish_odom_and_tf(now, yahboomcar_base_node::BodyVelocity{0.0, 0.0, 0.0});
      stale_zero_published_ = true;
      set_encoder_health(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Wheel encoders stale; odometry integration stopped");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr sensor_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  yahboomcar_base_node::MecanumParams mecanum_params_{0.0325, 0.080, 0.085};
  yahboomcar_base_node::OdomCovariances covariances_{
    0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001};
  double linear_scale_x_ = 1.0;
  double linear_scale_y_ = 1.0;
  double angular_scale_ = 1.0;
  double joint_state_timeout_ = 0.5;
  double max_wheel_delta_rad_ = 20.0;
  double x_pos_ = 0.0;
  double y_pos_ = 0.0;
  double heading_ = 0.0;
  bool pub_odom_tf_ = true;
  bool joint_data_received_ = false;
  bool stale_zero_published_ = false;
  bool has_prev_joints_ = false;
  uint8_t encoder_health_level_ = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  std::string encoder_health_message_ = "Initializing wheel encoder odometry";
  double prev_pos_fl_ = 0.0;
  double prev_pos_fr_ = 0.0;
  double prev_pos_bl_ = 0.0;
  double prev_pos_br_ = 0.0;
  std::string odom_frame_ = "odom";
  std::string base_footprint_frame_ = "base_footprint";
  std::vector<std::string> wheel_joint_names_;
  rclcpp::Time last_joint_time_;
  rclcpp::Time last_joint_receive_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
