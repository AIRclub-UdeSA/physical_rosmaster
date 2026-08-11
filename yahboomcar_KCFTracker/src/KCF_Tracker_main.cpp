#include <iostream>
#include "KCF_Tracker.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc,char **argv)
{
	rclcpp::init(argc, argv);
    std::cout<<"start"<<std::endl;
    rclcpp::spin(std::make_shared<ImageConverter>());
    return 0;
}
