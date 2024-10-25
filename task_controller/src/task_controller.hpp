#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "job/job.hpp"
#include "custom_msgs/msg/move_base_action_goal.hpp"

class TaskController : public rclcpp::Node {
 public:
  TaskController();

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  JobExecutor job_executor_;

  rclcpp::Publisher<custom_msgs::msg::MoveBaseActionGoal>::SharedPtr
      goal_publisher_;

  void timer_callback();
};
