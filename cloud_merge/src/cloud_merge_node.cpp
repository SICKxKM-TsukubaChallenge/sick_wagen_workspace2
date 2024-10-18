#include "cloud_merge.hpp"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pcl_tutorial::PointCloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}