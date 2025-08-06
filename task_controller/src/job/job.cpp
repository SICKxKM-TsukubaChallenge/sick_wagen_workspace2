#include "job.hpp"

#include <fstream>
#include <vector>

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


