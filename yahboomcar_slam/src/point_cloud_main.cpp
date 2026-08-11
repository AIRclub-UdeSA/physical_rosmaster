#include "point_cloud.h"
#include <thread> 

using namespace std;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudMapper>();
	thread t(&PointCloudMapper::viewer, node);  // 第二个参数必须填对象的this指针，否则会拷贝对象。
    rclcpp::spin(node);
    node->shutdown();
    rclcpp::shutdown();
    return 0;
}
