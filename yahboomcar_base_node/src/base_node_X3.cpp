#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
    this->declare_parameter<double>("wheelbase", 0.25);
    this->declare_parameter<double>("wheel_radius", 0.033);
    this->declare_parameter<double>("wheelbase_x", 0.160);
    this->declare_parameter<double>("wheelbase_y", 0.170);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
    this->declare_parameter<double>("linear_scale_x", 1.0);
    this->declare_parameter<double>("linear_scale_y", 1.0);
    this->declare_parameter<double>("angular_scale", 1.0);
    this->declare_parameter<bool>("pub_odom_tf", true);
    this->declare_parameter<bool>("use_joint_states", true);
    this->declare_parameter<double>("joint_state_timeout", 0.5);
    this->declare_parameter<double>("velocity_timeout", 0.5);
    this->declare_parameter<double>("max_wheel_delta_rad", 20.0);
    this->declare_parameter<double>("pose_covariance_x", 0.001);
    this->declare_parameter<double>("pose_covariance_y", 0.001);
    this->declare_parameter<double>("pose_covariance_yaw", 0.001);
    this->declare_parameter<double>("twist_covariance_x", 0.0001);
    this->declare_parameter<double>("twist_covariance_y", 0.0001);
    this->declare_parameter<double>("twist_covariance_yaw", 0.0001);

    this->get_parameter<double>("linear_scale_x", linear_scale_x_);
    this->get_parameter<double>("linear_scale_y", linear_scale_y_);
    this->get_parameter<double>("angular_scale", angular_scale_);
    this->get_parameter<double>("wheelbase", wheelbase_);
    this->get_parameter<double>("wheel_radius", mecanum_params_.wheel_radius);
    double wx = 0.160, wy = 0.170;
    this->get_parameter<double>("wheelbase_x", wx);
    this->get_parameter<double>("wheelbase_y", wy);
    mecanum_params_.lx = wx / 2.0;
    mecanum_params_.ly = wy / 2.0;

    this->get_parameter<bool>("pub_odom_tf", pub_odom_tf_);
    this->get_parameter<bool>("use_joint_states", use_joint_states_);
    this->get_parameter<std::string>("odom_frame", odom_frame_);
    this->get_parameter<std::string>("base_footprint_frame", base_footprint_frame_);
    this->get_parameter<double>("joint_state_timeout", joint_state_timeout_);
    this->get_parameter<double>("velocity_timeout", velocity_timeout_);
    this->get_parameter<double>("max_wheel_delta_rad", max_wheel_delta_rad_);
    this->get_parameter<double>("pose_covariance_x", covariances_.pose_x);
    this->get_parameter<double>("pose_covariance_y", covariances_.pose_y);
    this->get_parameter<double>("pose_covariance_yaw", covariances_.pose_yaw);
    this->get_parameter<double>("twist_covariance_x", covariances_.twist_x);
    this->get_parameter<double>("twist_covariance_y", covariances_.twist_y);
    this->get_parameter<double>("twist_covariance_yaw", covariances_.twist_yaw);

    const rclcpp::Time now = this->get_clock()->now();
    last_joint_time_ = now;
    last_joint_receive_time_ = now;
    previous_vel_time_ = now;
    last_vel_receive_time_ = now;
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      50,
      std::bind(&OdomPublisher::handle_joint_states, this, std::placeholders::_1));

    subscription_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "vel_raw",
      50,
      std::bind(&OdomPublisher::handle_vel, this, std::placeholders::_1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 50);
    sensor_watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&OdomPublisher::handle_sensor_timeout, this));
  }

private:
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
    if (!use_joint_states_ || msg->name.empty() || msg->position.empty()) {
      return;
    }

    int idx_fl = -1, idx_fr = -1, idx_bl = -1, idx_br = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto & name = msg->name[i];
      if (name.find("front_left") != std::string::npos) {
        idx_fl = i;
      } else if (name.find("front_right") != std::string::npos) {
        idx_fr = i;
      } else if (name.find("back_left") != std::string::npos) {
        idx_bl = i;
      } else if (name.find("back_right") != std::string::npos) {idx_br = i;}
    }

    if (idx_fl < 0 || idx_fr < 0 || idx_bl < 0 || idx_br < 0) {
      return;
    }

    const size_t largest_index = static_cast<size_t>(
      std::max(std::max(idx_fl, idx_fr), std::max(idx_bl, idx_br)));
    if (largest_index >= msg->position.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Ignoring malformed joint_states: wheel names exceed position array");
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
      return;
    }

    const double dt = (current_time - last_joint_time_).seconds();
    if (dt <= 0.0 || dt > joint_state_timeout_) {
      prev_pos_fl_ = pos_fl;
      prev_pos_fr_ = pos_fr;
      prev_pos_bl_ = pos_bl;
      prev_pos_br_ = pos_br;
      last_joint_time_ = current_time;
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
      return;
    }

    const yahboomcar_base_node::BodyVelocity raw_velocity =
      yahboomcar_base_node::compute_mecanum_body_velocity(deltas, mecanum_params_, dt);
    const yahboomcar_base_node::BodyVelocity velocity =
      yahboomcar_base_node::scale_body_velocity(
      raw_velocity, linear_scale_x_, linear_scale_y_, angular_scale_);

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
  }

  void handle_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const rclcpp::Time current_time = this->get_clock()->now();
    const bool velocity_was_fresh = velocity_data_received_ &&
      yahboomcar_base_node::is_source_fresh(
      (current_time - last_vel_receive_time_).seconds(), velocity_timeout_);
    const double vel_dt = velocity_was_fresh ?
      std::max(0.0, (current_time - previous_vel_time_).seconds()) : 0.0;
    last_vel_receive_time_ = current_time;
    previous_vel_time_ = current_time;
    velocity_data_received_ = true;

    const bool use_fresh_joint_states =
      yahboomcar_base_node::should_use_joint_states(
      use_joint_states_, joint_data_received_,
      (current_time - last_joint_receive_time_).seconds(), joint_state_timeout_);
    if (use_fresh_joint_states) {
      return;
    }

    const yahboomcar_base_node::BodyVelocity raw_velocity{
      msg->linear.x, msg->linear.y, msg->angular.z};
    const yahboomcar_base_node::BodyVelocity velocity =
      yahboomcar_base_node::scale_body_velocity(
      raw_velocity, linear_scale_x_, linear_scale_y_, angular_scale_);

    const yahboomcar_base_node::OdomState state =
      yahboomcar_base_node::integrate_velocity(
      yahboomcar_base_node::OdomState{x_pos_, y_pos_, heading_},
      velocity,
      vel_dt);

    x_pos_ = state.x;
    y_pos_ = state.y;
    heading_ = state.heading;

    publish_odom_and_tf(current_time, velocity);
    stale_zero_published_ = false;
  }

  void handle_sensor_timeout()
  {
    if (!joint_data_received_ && !velocity_data_received_) {
      return;
    }

    const rclcpp::Time now = this->get_clock()->now();
    const bool joint_states_fresh =
      yahboomcar_base_node::should_use_joint_states(
      use_joint_states_, joint_data_received_,
      (now - last_joint_receive_time_).seconds(), joint_state_timeout_);
    const bool velocity_fresh = velocity_data_received_ &&
      yahboomcar_base_node::is_source_fresh(
      (now - last_vel_receive_time_).seconds(), velocity_timeout_);

    if (!joint_states_fresh && !velocity_fresh && !stale_zero_published_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Odometry inputs timed out; publishing a zero twist without integrating pose");
      publish_odom_and_tf(now, yahboomcar_base_node::BodyVelocity{0.0, 0.0, 0.0});
      stale_zero_published_ = true;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_vel_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::TimerBase::SharedPtr sensor_watchdog_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  yahboomcar_base_node::MecanumParams mecanum_params_{0.033, 0.080, 0.085};
  yahboomcar_base_node::OdomCovariances covariances_{
    0.001, 0.001, 0.001, 0.0001, 0.0001, 0.0001};
  double linear_scale_x_ = 1.0;
  double linear_scale_y_ = 1.0;
  double angular_scale_ = 1.0;
  double joint_state_timeout_ = 0.5;
  double velocity_timeout_ = 0.5;
  double max_wheel_delta_rad_ = 20.0;
  double x_pos_ = 0.0;
  double y_pos_ = 0.0;
  double heading_ = 0.0;
  double wheelbase_ = 0.25;
  bool pub_odom_tf_ = true;
  bool use_joint_states_ = true;
  bool joint_data_received_ = false;
  bool velocity_data_received_ = false;
  bool stale_zero_published_ = false;
  bool has_prev_joints_ = false;
  double prev_pos_fl_ = 0.0;
  double prev_pos_fr_ = 0.0;
  double prev_pos_bl_ = 0.0;
  double prev_pos_br_ = 0.0;
  std::string odom_frame_ = "odom";
  std::string base_footprint_frame_ = "base_footprint";
  rclcpp::Time last_joint_time_;
  rclcpp::Time last_joint_receive_time_;
  rclcpp::Time previous_vel_time_;
  rclcpp::Time last_vel_receive_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
