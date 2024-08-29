#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "job/job.hpp"

class TaskController : public rclcpp::Node {
 public:
  TaskController();

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  JobExecutor job_executor_;

  void timer_callback();
};
