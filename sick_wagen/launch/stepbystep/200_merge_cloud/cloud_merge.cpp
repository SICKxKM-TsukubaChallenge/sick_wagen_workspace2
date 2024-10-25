#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CloudMergeNode : public rclcpp::Node
{
public:
    CloudMergeNode() : Node("cloud_merge_node")
    {
        // Initialize subscribers and publisher
        cloud1_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/merged_cloud", 10, std::bind(&CloudMergeNode::cloud1_callback, this, std::placeholders::_1));
        cloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/multiscan_xyz", 10, std::bind(&CloudMergeNode::cloud2_callback, this, std::placeholders::_1));
        merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merged_cloud", 10);

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void cloud1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        cloud1_ = *msg;
        merge_and_publish();
    }

    void cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        cloud2_ = *msg;
        merge_and_publish();
    }

    void merge_and_publish()
    {
        if (cloud1_.header.frame_id.empty() || cloud2_.header.frame_id.empty())
        {
            return;
        }

        // Transform clouds to the target frame
        sensor_msgs::msg::PointCloud2 cloud1_transformed, cloud2_transformed;
        try
        {
            cloud1_transformed = tf_buffer_->transform(cloud1_, target_frame_);
            cloud2_transformed = tf_buffer_->transform(cloud2_, target_frame_);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            return;
        }

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud1, pcl_cloud2, pcl_merged_cloud;
        pcl::fromROSMsg(cloud1_transformed, pcl_cloud1);
        pcl::fromROSMsg(cloud2_transformed, pcl_cloud2);

        // Merge clouds
        pcl_merged_cloud = pcl_cloud1 + pcl_cloud2;

        // Convert back to ROS message
        sensor_msgs::msg::PointCloud2 merged_cloud_msg;
        pcl::toROSMsg(pcl_merged_cloud, merged_cloud_msg);
        merged_cloud_msg.header.frame_id = target_frame_;
        merged_cloud_msg.header.stamp = this->now();

        // Publish merged cloud
        merged_cloud_pub_->publish(merged_cloud_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;

    sensor_msgs::msg::PointCloud2 cloud1_;
    sensor_msgs::msg::PointCloud2 cloud2_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string target_frame_ = "desired_frame";
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudMergeNode>());
    rclcpp::shutdown();
    return 0;
}