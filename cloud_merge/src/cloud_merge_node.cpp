#include "cloud_merge.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointcloudConcatenate>());
    rclcpp::shutdown();
    return 0;
}
