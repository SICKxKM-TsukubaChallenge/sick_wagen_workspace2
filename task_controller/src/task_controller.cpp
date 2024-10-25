#include "task_controller.hpp"


constexpr auto DT = 500;

TaskController::TaskController() : Node("task_controller") {
  // get ros2 parameters
  std::string csv_file;
  this->get_parameter("csv_file", csv_file);
  // create publisher
  goal_publisher_ =
      this->create_publisher<custom_msgs::msg::MoveBaseActionGoal>(
          "move_base/move/goal", rclcpp::QoS(10));

  // add jobs
  job_executor_.AddJob(Job("Start", Job::JobType::RUN, []() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start job");
    return true;
  }));
  job_executor_.AddJob(Job("Stop", Job::JobType::STOP, []() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stop job");
    return true;
  }));
  job_executor_.AddJob(Job("Adjust", Job::JobType::ADJUST, []() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Adjust job");
    return true;
  }));
  job_executor_.AddJob(Job("Perception", Job::JobType::PERCEPTION, []() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Perception job");
    return true;
  }));
  job_executor_.AddJob(Job("Run", Job::JobType::RUN, [this]() {
    custom_msgs::msg::MoveBaseActionGoal goal;
    goal_publisher_->publish(goal);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Run job");
    return true;
  }));

  // job_executor_.loadJobsFromCSV(csv_file);
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(DT),
                              std::bind(&TaskController::timer_callback, this));
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
