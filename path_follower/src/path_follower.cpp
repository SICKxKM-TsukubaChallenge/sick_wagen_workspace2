#include "path_follower.hpp"

PathFollowerNode::PathFollowerNode(const uint16_t& frequency)
    : Node("path_follower") {
  RCLCPP_INFO(get_logger(), "PathFollowerNode created");
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&PathFollowerNode::map_callback, this, std::placeholders::_1));
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
  predicted_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "predicted_path", 10);
  timer_ =
      create_wall_timer(std::chrono::milliseconds((int)(1 / frequency * 1000)),
                        std::bind(&PathFollowerNode::update, this));
}

void PathFollowerNode::update() {
  // run path following algorithm
  // update_path();
  // update_cmd_vel();
  test();
}

void PathFollowerNode::test() {
  RCLCPP_INFO(get_logger(), "Testing MPPIController");
  std::vector<MPPIController::singleSample> sample = mppi_.get_samples();
  // show x samples as visualization markers
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.resize(sample.size());
  for (uint32_t i = 0; i < sample.size(); i++) {
    markers.markers[i].header.frame_id = "map";
    markers.markers[i].header.stamp = now();
    markers.markers[i].ns = "mppi";
    markers.markers[i].id = i;
    markers.markers[i].type = visualization_msgs::msg::Marker::LINE_STRIP;
    markers.markers[i].action = visualization_msgs::msg::Marker::ADD;
    markers.markers[i].scale.x = 0.01;
    markers.markers[i].color.r = 0.0;
    markers.markers[i].color.g = 1.0;
    markers.markers[i].color.b = 0.0;
    markers.markers[i].color.a = 1.0;
    markers.markers[i].lifetime = rclcpp::Duration(std::chrono::seconds(1));
    markers.markers[i].points.resize(sample[i].x_.size());
    for (uint32_t j = 0; j < sample[i].x_.size(); j++) {
      markers.markers[i].points[j].x = sample[i].x_[j](0, 0);
      markers.markers[i].points[j].y = sample[i].x_[j](1, 0);
      markers.markers[i].points[j].z = 0.0;
    }
  }
  predicted_path_pub_->publish(markers);
}

void PathFollowerNode::update_cmd_vel() {
  RCLCPP_INFO(get_logger(), "Publishing cmd_vel and path");
  cmd_vel_pub_->publish(cmd_vel_);
  path_pub_->publish(path_);
}

void PathFollowerNode::update_path() {
  // run path planning algorithm
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
