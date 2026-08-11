#include <algorithm>
#include <functional>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "yahboomcar_base_node/odometry.hpp"

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
  : Node("base_node")
  {
    this->declare_parameter<double>("wheelbase", 0.25);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
    this->declare_parameter<double>("linear_scale_x", 1.0);
    this->declare_parameter<double>("linear_scale_y", 1.0);
    this->declare_parameter<bool>("pub_odom_tf", true);

    this->get_parameter<double>("linear_scale_x", linear_scale_x_);
    this->get_parameter<double>("linear_scale_y", linear_scale_y_);
    this->get_parameter<double>("wheelbase", wheelbase_);
    this->get_parameter<bool>("pub_odom_tf", pub_odom_tf_);
    this->get_parameter<std::string>("odom_frame", odom_frame_);
    this->get_parameter<std::string>("base_footprint_frame", base_footprint_frame_);

    last_vel_time_ = this->get_clock()->now();
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "vel_raw",
      50,
      std::bind(&OdomPublisher::handle_vel, this, std::placeholders::_1));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 50);
  }

private:
  void handle_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const rclcpp::Time current_time = this->get_clock()->now();
    const double vel_dt = std::max(0.0, (current_time - last_vel_time_).seconds());
    last_vel_time_ = current_time;

    const yahboomcar_base_node::BodyVelocity velocity{
      msg->linear.x * linear_scale_x_,
      msg->linear.y * linear_scale_y_,
      msg->angular.z};
    const yahboomcar_base_node::OdomState state =
      yahboomcar_base_node::integrate_velocity(
      yahboomcar_base_node::OdomState{x_pos_, y_pos_, heading_},
      velocity,
      vel_dt);

    x_pos_ = state.x;
    y_pos_ = state.y;
    heading_ = state.heading;

    const builtin_interfaces::msg::Time stamp = current_time;
    const nav_msgs::msg::Odometry odom =
      yahboomcar_base_node::make_odometry_msg(
      stamp,
      odom_frame_,
      base_footprint_frame_,
      state,
      velocity);
    odom_publisher_->publish(odom);

    if (pub_odom_tf_) {
      const geometry_msgs::msg::TransformStamped transform =
        yahboomcar_base_node::make_odom_transform(
        stamp,
        odom_frame_,
        base_footprint_frame_,
        state);
      tf_broadcaster_->sendTransform(transform);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double linear_scale_x_ = 0.0;
  double linear_scale_y_ = 0.0;
  double x_pos_ = 0.0;
  double y_pos_ = 0.0;
  double heading_ = 0.0;
  double wheelbase_ = 0.25;
  bool pub_odom_tf_ = false;
  std::string odom_frame_ = "odom";
  std::string base_footprint_frame_ = "base_footprint";
  rclcpp::Time last_vel_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
