//ros .h
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "yahboomcar_msgs/msg/point_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

//common .h
#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>


class mediapipeCloud:public rclcpp ::Node
{

   rclcpp::Subscription<yahboomcar_msgs::msg::PointArray>::SharedPtr subscription_;
   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_;
   
	public:
	  mediapipeCloud()
	  : Node("mediapipe_point")
	  {            
	  	subscription_ = this->create_subscription<yahboomcar_msgs::msg::PointArray>("/mediapipe/points",50,std::bind(&mediapipeCloud::handle_point,this,std::placeholders::_1));
	  	pub_pcl_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mediapipe_cloud", 1000);
	  	
	  }
	  	private:
	  	  void handle_point(const std::shared_ptr<yahboomcar_msgs::msg::PointArray> msg)
	  	  {
            
            //std::cout << msg->points.size()<<std::endl;
            unsigned int num_points = msg->points.size();
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            cloud.points.resize(num_points);
            sensor_msgs::msg::PointCloud2 output_msg;
            output_msg.header.stamp = rclcpp::Clock().now();
            for (int i = 0; i < msg->points.size(); i++)
            {
            	std::cout << msg->points[i].x<<std::endl;
            	geometry_msgs::msg::Point Point;
            	cloud.points[i].x = msg->points[i].x;
            	cloud.points[i].y = msg->points[i].y;
            	cloud.points[i].z = msg->points[i].z;
            	cloud.points[i].r = 0;
            	cloud.points[i].g = 255;
            	cloud.points[i].b = 0;
            }
            pcl::toROSMsg(cloud, output_msg);
            output_msg.header.frame_id = "map";
            pub_pcl_->publish(output_msg);
		  }

};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<mediapipeCloud>());
	rclcpp::shutdown();
    return 0;
}


