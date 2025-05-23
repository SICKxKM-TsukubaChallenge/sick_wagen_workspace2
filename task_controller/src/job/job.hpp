#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class Job {
 public:
  enum class JobState { IDLE, RUNNING, FINISHED };
  enum class JobType { RUN, ADJUST, STOP, PERCEPTION };
  Job(const std::string& name, JobType type, std::function<bool()> job_function)
      : name_(name),
        type_(type),
        job_function_(job_function),
        state_(JobState::IDLE) {}

  JobState getState() const { return state_; }
  JobType getType() const { return type_; }
  std::string getName() const { return name_; }
  bool doJobFunction() {
    if (job_function_()) {
      state_ = JobState::FINISHED;
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Job %s finished",
                  name_.c_str());
      return true;
    } else {
      state_ = JobState::RUNNING;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Job %s running",
                  name_.c_str());
      return false;
    }
  }

 private:
  std::string name_;
  JobType type_;
  std::function<bool()> job_function_;
  JobState state_;
};

class JobExecutor {
 private:
  std::vector<Job> jobs_;
  bool is_running_ = false;

 public:
  void AddJob(const Job& job) { jobs_.push_back(job); }
  void ExecuteJobs();
};
