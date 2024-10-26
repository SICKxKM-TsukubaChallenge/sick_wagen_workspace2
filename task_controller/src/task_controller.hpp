#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "job/job.hpp"

class TaskController : public rclcpp::Node {
 public:
  TaskController();

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  JobExecutor job_executor_;

  rclcpp::Publisher<custom_msgs::msg::MoveBaseActionGoal>::SharedPtr
      goal_publisher_;

  void timer_callback();
  void loadJobsFromCSV(const std::string& csv_file);
};
