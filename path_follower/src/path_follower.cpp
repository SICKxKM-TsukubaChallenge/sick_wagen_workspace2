#include "path_follower.hpp"

PathFollowerNode::PathFollowerNode(const uint16_t& frequency)
    : Node("path_follower") {
  RCLCPP_INFO(get_logger(), "PathFollowerNode created");
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", 10,
      std::bind(&PathFollowerNode::map_callback, this, std::placeholders::_1));
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
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

void PathFollowerNode::test() { RCLCPP_INFO(get_logger(), "Test"); }

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