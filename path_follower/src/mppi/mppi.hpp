#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace Eigen;

class MPPIController {
 public:
  MPPIController(uint32_t num_samples, uint32_t num_timesteps,
                 MatrixXf cov_matrix, std::vector<double> u_min,
                 std::vector<double> u_max, double lambda, double dt,
                 uint16_t map_width, uint16_t map_height,
                 uint8_t control_dimension, double map_resolution,
                 MatrixXf transition_matrix);
  ~MPPIController();
  struct singleSample {
    std::vector<MatrixXf> v_;  // input sample
    std::vector<MatrixXf> x_;  // predicted state
  };

  std::vector<singleSample> get_samples() { return samples_; };

 private:
  // Parameters
  const uint32_t num_samples_;
  const uint32_t num_timesteps_;
  const MatrixXf cov_matrix_;
  const std::vector<double> u_min_;
  const std::vector<double> u_max_;
  const double lambda_;
  const double dt_;                      // seconds
  const uint16_t map_width_;             // pixels
  const uint16_t map_height_;            // pixels
  const uint8_t control_dimension_ = 2;  // x and y
  const double map_resolution_;          // meters per pixel
  const MatrixXf transition_matrix_ =
      MatrixXf::Identity(control_dimension_, control_dimension_);

  // Variables
  MatrixXf current_pose_;    // current pose
  std::vector<MatrixXf> u_;  // input
  std::vector<singleSample> samples_;
  std::vector<MatrixXf> prev_u_;  // previous control input list
  uint8_t input_number;

  // Functions
  void gaussian_sampling(const std::vector<MatrixXf>& u);
  void calc_predicted_state();
};