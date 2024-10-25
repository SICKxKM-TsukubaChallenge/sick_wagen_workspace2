#include "job.hpp"

#include <fstream>

void JobExecutor::ExecuteJobs() {
  if (jobs_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("JobExecuter"), "No jobs to execute");
    return;
  }
  if (is_running_) {
    return;
  } else {
    if (jobs_[0].doJobFunction()) {
      jobs_.erase(jobs_.begin());
      is_running_ = false;
    } else {
      is_running_ = true;
    }
  }
}

void JobExecutor::loadJobsFromCSV(const std::string& csv_file) {
  std::ifstream file(csv_file);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("JobExecuter"), "Could not open file %s",
                 csv_file.c_str());
    return;
  }
  std::string line;
  while (std::getline(file, line)) {
    // get last element from line (job type)
    std::string job_type_str = line.substr(line.find_last_of(",") + 1);
    if (job_type_str == "RUN") {
    } else if (job_type_str == "ADJUST") {
    } else if (job_type_str == "STOP") {
    } else if (job_type_str == "PERCEPTION") {
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("JobExecuter"), "Unknown job type %s",
                   job_type_str.c_str());
      continue;
    }
  }
}
