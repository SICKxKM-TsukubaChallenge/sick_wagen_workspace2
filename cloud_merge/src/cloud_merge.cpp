#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdlib>  // std::getenv

class PointcloudConcatenate : public rclcpp::Node {
public:
    PointcloudConcatenate(const rclcpp::NodeOptions &options) : Node("pointcloud_concatenate", options) {
        // パラメータの宣言とデフォルト値
        this->declare_parameter<std::string>("cloud_in1_topic", "cloud_in1");
        this->declare_parameter<std::string>("cloud_in2_topic", "cloud_in2");
        this->declare_parameter<std::string>("cloud_in3_topic", "cloud_in3");
        this->declare_parameter<std::string>("cloud_in4_topic", "cloud_in4");
        this->declare_parameter<std::string>("cloud_out_topic", "cloud_out");
        this->declare_parameter<std::string>("cloud_in1_frame", "frame1");
        this->declare_parameter<std::string>("cloud_in2_frame", "frame2");
        this->declare_parameter<std::string>("cloud_in3_frame", "frame3");
        this->declare_parameter<std::string>("cloud_in4_frame", "frame4");

        // パラメータの取得
        std::string cloud_in1_topic = this->get_parameter("cloud_in1_topic").as_string();
        std::string cloud_in2_topic = this->get_parameter("cloud_in2_topic").as_string();
        std::string cloud_in3_topic = this->get_parameter("cloud_in3_topic").as_string();
        std::string cloud_in4_topic = this->get_parameter("cloud_in4_topic").as_string();
        std::string cloud_out_topic = this->get_parameter("cloud_out_topic").as_string();
        cloud_in1_frame = this->get_parameter("cloud_in1_frame").as_string();
        cloud_in2_frame = this->get_parameter("cloud_in2_frame").as_string();
        cloud_in3_frame = this->get_parameter("cloud_in3_frame").as_string();
        cloud_in4_frame = this->get_parameter("cloud_in4_frame").as_string();

        // サブスクライバとパブリッシャの設定
        sub_cloud_in1 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in1_topic, 10, std::bind(&PointcloudConcatenate::subCallbackCloudIn1, this, std::placeholders::_1));
        sub_cloud_in2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in2_topic, 10, std::bind(&PointcloudConcatenate::subCallbackCloudIn2, this, std::placeholders::_1));
        sub_cloud_in3 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in3_topic, 10, std::bind(&PointcloudConcatenate::subCallbackCloudIn3, this, std::placeholders::_1));
        sub_cloud_in4 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_in4_topic, 10, std::bind(&PointcloudConcatenate::subCallbackCloudIn4, this, std::placeholders::_1));

        pub_cloud_out = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_out_topic, 10);

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

        try {
            if (cloud_in1_received) {
                auto transform = tfBuffer->lookupTransform(cloud_in1_frame, cloud_in1.header.frame_id, tf2::TimePointZero);
                sensor_msgs::msg::PointCloud2 transformed_cloud;
                pcl_ros::transformPointCloud(cloud_in1_frame, transform, cloud_in1, transformed_cloud);
                pcl::fromROSMsg(transformed_cloud, pcl_cloud_in1);
            }
            if (cloud_in2_received) {
                auto transform = tfBuffer->lookupTransform(cloud_in2_frame, cloud_in2.header.frame_id, tf2::TimePointZero);
                sensor_msgs::msg::PointCloud2 transformed_cloud;
                pcl_ros::transformPointCloud(cloud_in2_frame, transform, cloud_in2, transformed_cloud);
                pcl::fromROSMsg(transformed_cloud, pcl_cloud_in2);
            }
            if (cloud_in3_received) {
                auto transform = tfBuffer->lookupTransform(cloud_in3_frame, cloud_in3.header.frame_id, tf2::TimePointZero);
                sensor_msgs::msg::PointCloud2 transformed_cloud;
                pcl_ros::transformPointCloud(cloud_in3_frame, transform, cloud_in3, transformed_cloud);
                pcl::fromROSMsg(transformed_cloud, pcl_cloud_in3);
            }
            if (cloud_in4_received) {
                auto transform = tfBuffer->lookupTransform(cloud_in4_frame, cloud_in4.header.frame_id, tf2::TimePointZero);
                sensor_msgs::msg::PointCloud2 transformed_cloud;
                pcl_ros::transformPointCloud(cloud_in4_frame, transform, cloud_in4, transformed_cloud);
                pcl::fromROSMsg(transformed_cloud, pcl_cloud_in4);
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            return;
        }

        pcl_cloud_out = pcl_cloud_in1;
        pcl_cloud_out += pcl_cloud_in2;
        pcl_cloud_out += pcl_cloud_in3;
        pcl_cloud_out += pcl_cloud_in4;

        pcl::toROSMsg(pcl_cloud_out, cloud_out);
        cloud_out.header.frame_id = "base_link";  // 統合後のフレームIDを設定
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
    bool cloud_in2_received = false;
    bool cloud_in3_received = false;
    bool cloud_in4_received = false;

    std::string cloud_in1_frame;
    std::string cloud_in2_frame;
    std::string cloud_in3_frame;
    std::string cloud_in4_frame;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // 環境変数からYAMLファイルのパスを取得
    const char *config_path_env = std::getenv("CLOUD_MERGE_CONFIG_PATH");
    std::string config_path = config_path_env ? config_path_env : "config/cloud_merge_params.yaml";

    // YAMLファイルを読み込む
    rclcpp::NodeOptions options;
    options.parameter_overrides_from_file(config_path);

    RCLCPP_INFO(rclcpp::get_logger("PointcloudConcatenate"), "Using config file: %s", config_path.c_str());

    rclcpp::spin(std::make_shared<PointcloudConcatenate>(options));
    rclcpp::shutdown();
    return 0;
}