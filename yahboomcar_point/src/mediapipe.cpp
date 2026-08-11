#include "yahboomcar_msgs/msg/point_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <geometry_msgs/msg/point.h>


#include <memory>78
#include <string>
using std::placeholders::_1;

class mediapipeCloud:public rclcpp ::Node
{

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl;
    rclcpp::Subscription<yahboomcar_msgs::msg::PointArray>::SharedPtr sub_point;
    bool viewer_display= true;
    //pcl::visualization::CloudViewer viewer;

public:
	mediapipeCloud()
	:Node("mediapipe_point")
    {
        pub_pcl = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mediapipe_cloud", 1000);
        sub_point = this->create_subscription<yahboomcar_msgs::msg::PointArray>("/mediapipe/points",50,std::bind(&mediapipeCloud::handle_point,this,_1));
    }
private:
    void handle_point(const std::shared_ptr<yahboomcar_msgs::msg::PointArray > msg)
    {
        std::cout<<"I got it"<<std::endl;
        //std::cout<<"msg: ",msg->points[0]<<endl;
        /*unsigned int num_points = msg.points.size();
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.points.resize(num_points);
        sensor_msgs::msg::PointCloud2 output_msg;
        output_msg.header.stamp = rclcpp::Clock().now();
        for (int i = 0; i < msg.points.size(); i++)
        {
            geometry_msgs::msg::Point Point;
            Point = msg.points[i];
            cloud.points[i].x = Point.x;
            cloud.points[i].y = Point.y;
            cloud.points[i].z = Point.z;
            cloud.points[i].r = 0;
            cloud.points[i].g = 255;
            cloud.points[i].b = 0;
        }
        pcl::toROSMsg(cloud, output_msg);
        output_msg.header.frame_id = "map";
        pub_pcl->publish(output_msg);*/
        
        /*if(viewer_display)
        {
            viewer.showCloud(cloud.makeShared());
        }*/
    }   
        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::cout<<"start it"<<std::endl;
    rclcpp::spin(std::make_shared<mediapipeCloud>());
    rclcpp::shutdown();
    return 0;
}
