#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class PointcloudConcatenate : public rclcpp::Node {
public:
    PointcloudConcatenate() : Node("pointcloud_concatenate") {
        sub_cloud_in1 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud_in1", 10, std::bind(&PointcloudConcatenate::subCallbackCloudIn1, this, std::placeholders::_1));
        sub_cloud_in2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud_in2", 10, std::bind(&PointcloudConcatenate::subCallbackCloudIn2, this, std::placeholders::_1));
        sub_cloud_in3 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud_in3", 10, std::bind(&PointcloudConcatenate::subCallbackCloudIn3, this, std::placeholders::_1));
        sub_cloud_in4 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud_in4", 10, std::bind(&PointcloudConcatenate::subCallbackCloudIn4, this, std::placeholders::_1));

        pub_cloud_out = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 10);

        tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
    }

private:
    void subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_in1 = *msg;
        cloud_in1_received = true;
        concatenatePointClouds();
    }

    void subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_in2 = *msg;
        cloud_in2_received = true;
        concatenatePointClouds();
    }

    void subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_in3 = *msg;
        cloud_in3_received = true;
        concatenatePointClouds();
    }

    void subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        cloud_in4 = *msg;
        cloud_in4_received = true;
        concatenatePointClouds();
    }

    void concatenatePointClouds() {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in1, pcl_cloud_in2, pcl_cloud_in3, pcl_cloud_in4, pcl_cloud_out;

        if (cloud_in1_received) {
            pcl::fromROSMsg(cloud_in1, pcl_cloud_in1);
        }
        if (cloud_in2_received) {
            pcl::fromROSMsg(cloud_in2, pcl_cloud_in2);
        }
        if (cloud_in3_received) {
            pcl::fromROSMsg(cloud_in3, pcl_cloud_in3);
        }
        if (cloud_in4_received) {
            pcl::fromROSMsg(cloud_in4, pcl_cloud_in4);
        }

        pcl_cloud_out = pcl_cloud_in1;
        pcl_cloud_out += pcl_cloud_in2;
        pcl_cloud_out += pcl_cloud_in3;
        pcl_cloud_out += pcl_cloud_in4;

        pcl::toROSMsg(pcl_cloud_out, cloud_out);
        pub_cloud_out->publish(cloud_out);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in1;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in2;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in3;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in4;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_out;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    sensor_msgs::msg::PointCloud2 cloud_in1;
    sensor_msgs::msg::PointCloud2 cloud_in2;
    sensor_msgs::msg::PointCloud2 cloud_in3;
    sensor_msgs::msg::PointCloud2 cloud_in4;
    sensor_msgs::msg::PointCloud2 cloud_out;

    bool cloud_in1_received = false;
    bool cloud_in1_received_recent = false;
    bool cloud_in2_received = false;
    bool cloud_in2_received_recent = false;
    bool cloud_in3_received = false;
    bool cloud_in3_received_recent = false;
    bool cloud_in4_received = false;
    bool cloud_in4_received_recent = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointcloudConcatenate>());
    rclcpp::shutdown();
    return 0;
}