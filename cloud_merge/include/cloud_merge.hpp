#ifndef CLOUD_MERGE_HPP
#define CLOUD_MERGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/concatenate.h>

class PointcloudConcatenate : public rclcpp::Node {
public:
    PointcloudConcatenate();
    ~PointcloudConcatenate();

private:
    void subCallbackCloudIn1(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void subCallbackCloudIn2(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void subCallbackCloudIn3(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void subCallbackCloudIn4(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void handleParams();
    double getHz();
    void update();
    void publishPointcloud(sensor_msgs::msg::PointCloud2 cloud);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in1;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in2;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in3;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in4;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_out;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    std::string param_frame_target_;
    int param_clouds_;
    double param_hz_;

    sensor_msgs::msg::PointCloud2 cloud_in1;
    sensor_msgs::msg::PointCloud2 cloud_in2;
    sensor_msgs::msg::PointCloud2 cloud_in3;
    sensor_msgs::msg::PointCloud2 cloud_in4;
    sensor_msgs::msg::PointCloud2 cloud_out;

    bool cloud_in1_received = false;
    bool cloud_in2_received = false;
    bool cloud_in3_received = false;
    bool cloud_in4_received = false;
};

#endif // CLOUD_MERGE_HPP
