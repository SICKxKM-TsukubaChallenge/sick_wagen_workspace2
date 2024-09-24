#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mppi/mppi.hpp"

class PathFollowerNode : public rclcpp::Node {
 public:
  PathFollowerNode(const uint16_t& frequency = 1.0);

 private:
  geometry_msgs::msg::Twist cmd_vel_;
  nav_msgs::msg::OccupancyGrid newest_map_;
  nav_msgs::msg::Path path_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      predicted_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  MPPIController mppi_{100,        10,           MatrixXf::Identity(2, 2),
                       {0.0, 0.0}, {10.0, 10.0}, 1.0,
                       1,          100,          100,
                       2,          0.1,          MatrixXf::Identity(2, 2)};
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    newest_map_ = *msg;
  }
  void update();
  void update_path();
  void update_cmd_vel();
  void test();
};