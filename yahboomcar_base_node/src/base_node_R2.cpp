#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
//#include <turtlesim/msg/pose.hpp>

#include <memory>
#include <string>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class OdomPublisher:public rclcpp ::Node
{
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
   double linear_scale_x_ = 0.0 ;
   double linear_scale_y_ = 0.0;
   double vel_dt_ = 0.0;
   double x_pos_ = 0.0;
   double y_pos_ = 0.0;
   double heading_ = 0.0;
   double linear_velocity_x_ = 0.0;
   double linear_velocity_y_ = 0.0;
   double wheelbase_ = 0.25;
   bool pub_odom_tf_ = false;
   rclcpp::Time last_vel_time_  ;
	public:
	  OdomPublisher()
	  : Node("base_node")
	  {            
            this->declare_parameter<double>("wheelbase",0.25);
            this->declare_parameter<std::string>("odom_frame","odom");
            this->declare_parameter<std::string>("base_footprint_frame","base_footprint"); 
            this->declare_parameter<double>("linear_scale_x",1.0);
            this->declare_parameter<double>("linear_scale_y",1.0);
            this->declare_parameter<bool>("pub_odom_tf",true);

            this->get_parameter<double>("linear_scale_x",linear_scale_x_);
            this->get_parameter<double>("linear_scale_y",linear_scale_y_);
            this->get_parameter<double>("wheelbase",wheelbase_);
            this->get_parameter<bool>("pub_odom_tf",pub_odom_tf_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	  	subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("vel_raw",50,std::bind(&OdomPublisher::handle_vel,this,_1));
	  	odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 50);
	  	}
	  	private:
	  	  void handle_vel(const std::shared_ptr<geometry_msgs::msg::Twist > msg)
	  	  {
            //geometry_msgs::msg::Twist twist;
                       
	  	  	rclcpp::Time curren_time = rclcpp::Clock().now();
	  	  	linear_velocity_x_ = msg->linear.x * linear_scale_x_;// scale = 1
    		linear_velocity_y_ = msg->linear.y * linear_scale_y_;
			vel_dt_ = (curren_time - last_vel_time_).seconds();
            //std::cout<<"curren_time: "<<curren_time.seconds()<<std::endl;
           // std::cout<<"vel_dt: "<<vel_dt_<<std::endl;
    		last_vel_time_ = curren_time;
			double steer_angle = linear_velocity_y_;
			double MI_PI = 3.1416;
			double R = wheelbase_ / tan(steer_angle/180.0*MI_PI);
			double angular_velocity_z_ = linear_velocity_x_/R;
    		double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    		double delta_x = (linear_velocity_x_ * cos(heading_)) * vel_dt_; //m
    		double delta_y = (linear_velocity_x_ * sin(heading_)) * vel_dt_; //m	
 			x_pos_ += delta_x;
    		y_pos_ += delta_y;
			heading_ += delta_heading;
            
            tf2::Quaternion myQuaternion;
			geometry_msgs::msg::Quaternion odom_quat ; 
			myQuaternion.setRPY(0.00,0.00,heading_ );

            odom_quat.x = myQuaternion.x();
            odom_quat.y = myQuaternion.y();
            odom_quat.z = myQuaternion.z();
            odom_quat.w = myQuaternion.w();
  
			nav_msgs::msg::Odometry odom;
			odom.header.stamp = curren_time;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_footprint";
			// robot's position in x,y and z
			odom.pose.pose.position.x = x_pos_;
			odom.pose.pose.position.y = y_pos_;
			odom.pose.pose.position.z = 0.0;
			// robot's heading in quaternion

            odom.pose.pose.orientation = odom_quat;
			odom.pose.covariance[0] = 0.001;
			odom.pose.covariance[7] = 0.001;
			odom.pose.covariance[35] = 0.001;
			// linear speed from encoders
			odom.twist.twist.linear.x = linear_velocity_x_;
			//odom.twist.twist.linear.y = linear_velocity_y_;
			odom.twist.twist.linear.y = 0.0; // vy = 0.0
			odom.twist.twist.linear.z = 0.0;
			odom.twist.twist.angular.x = 0.0;
			odom.twist.twist.angular.y = 0.0;
			// angular speed from encoders
			odom.twist.twist.angular.z = angular_velocity_z_;
			odom.twist.covariance[0] = 0.0001;
			odom.twist.covariance[7] = 0.0001;
			odom.twist.covariance[35] = 0.0001;
			// ROS_INFO("ODOM PUBLISH");	
			odom_publisher_ -> publish(odom);
            if (pub_odom_tf_)
            {
            
                geometry_msgs::msg::TransformStamped t;
                rclcpp::Time now = this->get_clock()->now();
                t.header.stamp = now;
                t.header.frame_id = "odom";
                t.child_frame_id = "base_footprint";
                t.transform.translation.x = x_pos_;
                t.transform.translation.y = y_pos_;
                t.transform.translation.z = 0.0;
                
                t.transform.rotation.x = myQuaternion.x();
                t.transform.rotation.y = myQuaternion.y();
                t.transform.rotation.z = myQuaternion.z();
                t.transform.rotation.w = myQuaternion.w();
                
                tf_broadcaster_->sendTransform(t);
                  
            }
		  	  }

};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdomPublisher>());
	rclcpp::shutdown();
    return 0;
}

