#include "task_controller.hpp"

TaskController::TaskController() : Node("task_controller") {
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&TaskController::timer_callback, this));
  job_executor_.AddJob(Job("Start", Job::JobType::RUN, []() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start job");
    return true;
  }));
}

void TaskController::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Timer callback");
  job_executor_.ExecuteJobs();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskController>());
  rclcpp::shutdown();
  return 0;
}
