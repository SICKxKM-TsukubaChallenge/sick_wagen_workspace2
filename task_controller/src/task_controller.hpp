#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "custom_msgs/msg/move_base_action_goal.hpp"
#include "custom_msgs/msg/move_base_action_result.hpp"
#include "job/job.hpp"

class TaskController : public rclcpp::Node {
 public:
  TaskController();

 private:
  bool resultInQue = false;
  rclcpp::TimerBase::SharedPtr timer_;
  JobExecutor job_executor_;

  rclcpp::Publisher<custom_msgs::msg::MoveBaseActionGoal>::SharedPtr
      goal_publisher_;
  rclcpp::Subscription<custom_msgs::msg::MoveBaseActionResult>::SharedPtr
      result_subscription_;

  void timer_callback();
  void resultCallback(
      const custom_msgs::msg::MoveBaseActionResult::SharedPtr msg);
  void loadJobsFromCSV(const std::string& csv_file);
};
