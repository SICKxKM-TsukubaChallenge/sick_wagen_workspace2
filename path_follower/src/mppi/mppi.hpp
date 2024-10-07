#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace Eigen;

/**
 * @brief MPPIController class
 * @details MPPIController class for path following
 * @param num_samples Number of samples
 * @param num_timesteps Number of timesteps
 * @param cov_matrix Covariance matrix
 * @param u_min Minimum control input vector
 * @param u_max Maximum control input vector
 * @param lambda Lambda
 * @param dt Time step in seconds
 * @param map_width Map width in pixels
 * @param map_height Map height in pixels
 * @param control_dimension Control dimension
 * @param input_dimension Input dimension
 * @param map_resolution Map resolution in meters per pixel
 * @param transition_matrix Transition matrix
 */
class MPPIController {
 public:
  MPPIController(uint32_t num_samples, uint32_t num_timesteps,
                 MatrixXf cov_matrix, std::vector<double> u_min,
                 std::vector<double> u_max, double lambda, double dt,
                 uint16_t map_width, uint16_t map_height,
                 uint8_t control_dimension, uint8_t input_dimension,
                 double map_resolution);
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
  const double dt_;            // seconds
  const uint16_t map_width_;   // pixels
  const uint16_t map_height_;  // pixels
  // robot position is always at the center of the map
  const uint8_t pose_dimension_ = 3;   // x ,y and theta
  const uint8_t input_dimension_ = 2;  // linear and angular velocity
  // for this above two parameters, actually we did not need them. We can get it
  // from the matrix size, but to prevent any error, we define them.
  const double map_resolution_;  // meters per pixel

  // Variables
  MatrixXf current_pose_;    // current pose
  std::vector<MatrixXf> u_;  // input
  std::vector<singleSample> samples_;
  std::vector<MatrixXf> prev_u_;  // previous control input list
  uint8_t input_number;
  std::vector<double> robot_center_cost_map_;

  // Functions
  void gaussian_sampling(const std::vector<MatrixXf>& u);
  void calc_predicted_state();
  MatrixXf diff_robot_transition_matrix(const MatrixXf& x);
  double calc_cost(const singleSample& sample);

  // Getter and setter
  void set_current_pose(const MatrixXf& current_pose) {
    current_pose_ = current_pose;
  }
  MatrixXf get_current_pose() { return current_pose_; };
  std::vector<MatrixXf> get_u() { return u_; };
};