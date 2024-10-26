#include "task_controller.hpp"

#include <fstream>

constexpr auto DT = 500;
const std::string BASE_LINK = "base_link";

TaskController::TaskController() : Node("task_controller") {
  // get ros2 parameters
  std::string csv_file;
  // this->get_parameter("csv_file", csv_file);
  csv_file =
      "/home/curious/tsukuba_ws/src/sick_wagen_workspace2/task_controller/"
      "config/20240928_114854.csv";
  // create publisher
  goal_publisher_ =
      this->create_publisher<custom_msgs::msg::MoveBaseActionGoal>(
          "move_base/move/goal", rclcpp::QoS(10));

  // add jobs
  job_executor_.AddJob(Job("Start", Job::JobType::RUN, []() {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start job");
    return true;
  }));

  loadJobsFromCSV(csv_file);
  // job_executor_.AddJob(Job("Stop", Job::JobType::STOP, []() {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stop job");
  //   return true;
  // }));
  // job_executor_.AddJob(Job("Adjust", Job::JobType::ADJUST, []() {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Adjust job");
  //   return true;
  // }));
  // job_executor_.AddJob(Job("Perception", Job::JobType::PERCEPTION, []() {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Perception job");
  //   return true;
  // }));
  // job_executor_.AddJob(Job("Run", Job::JobType::RUN, [this]() {
  //   custom_msgs::msg::MoveBaseActionGoal goal;
  //   goal_publisher_->publish(goal);
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Run job");
  //   return true;
  // }));

  // job_executor_.loadJobsFromCSV(csv_file);
  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(DT),
                              std::bind(&TaskController::timer_callback, this));
}

void TaskController::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Timer callback");
  job_executor_.ExecuteJobs();
}

void TaskController::loadJobsFromCSV(const std::string& csv_file) {
  std::ifstream file(csv_file);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("JobExecuter"), "Could not open file %s",
                 csv_file.c_str());
    return;
  }

  std::string line;
  uint8_t goalCount = 0;

  while (std::getline(file, line)) {
    uint8_t job_type = std::stoi(line.substr(line.find_last_of(",") + 1));

    if (job_type == 0) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "append Run job");
      custom_msgs::msg::MoveBaseActionGoal goal;
      goal.header.stamp = this->now();
      goal.header.frame_id = BASE_LINK;
      goal.goal_id.id = std::to_string(goalCount);
      goal.goal_id.stamp = this->now();
      goal.goal.target_pose.header.frame_id = BASE_LINK;
      goal.goal.target_pose.header.stamp = this->now();
      // first column is x
      goal.goal.target_pose.pose.position.x =
          std::stof(line.substr(0, line.find(",")));
      // remove x from line
      line = line.substr(line.find(",") + 1);
      // first column is y
      goal.goal.target_pose.pose.position.y =
          std::stof(line.substr(0, line.find(",")));
      // remove y from line
      line = line.substr(line.find(",") + 1);
      // first column is z
      goal.goal.target_pose.pose.position.z =
          std::stof(line.substr(0, line.find(",")));
      // remove z from line
      line = line.substr(line.find(",") + 1);

      // first column is qx
      goal.goal.target_pose.pose.orientation.x =
          std::stof(line.substr(0, line.find(",")));
      // remove qx from line
      line = line.substr(line.find(",") + 1);
      // first column is qy
      goal.goal.target_pose.pose.orientation.y =
          std::stof(line.substr(0, line.find(",")));
      // remove qy from line
      line = line.substr(line.find(",") + 1);
      // first column is qz
      goal.goal.target_pose.pose.orientation.z =
          std::stof(line.substr(0, line.find(",")));
      // remove qz from line
      line = line.substr(line.find(",") + 1);
      // first column is qw
      goal.goal.target_pose.pose.orientation.w =
          std::stof(line.substr(0, line.find(",")));
      // remove qw from line
      line = line.substr(line.find(",") + 1);

      job_executor_.AddJob(Job("Run", Job::JobType::RUN, [this, goal]() {
        goal_publisher_->publish(goal);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RUN job from csv");
        return true;
      }));
      goalCount++;
    } else if (job_type == 1) {
      // adjust job
    } else if (job_type == 2) {
      // stop job
    } else if (job_type == 3) {
      // perception job
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("JobExecuter"), "Unknown job type %d",
                   job_type);
      continue;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loaded total %d jobs", goalCount);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskController>());
  rclcpp::shutdown();
  return 0;
}
