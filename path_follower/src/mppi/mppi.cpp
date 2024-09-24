#include "mppi.hpp"

#include <random>
#include <rclcpp/rclcpp.hpp>

MPPIController::MPPIController(uint32_t num_samples, uint32_t num_timesteps,
                               MatrixXf cov_matrix, std::vector<double> u_min,
                               std::vector<double> u_max, double lambda,
                               double dt, uint16_t map_width,
                               uint16_t map_height, uint8_t control_dimension,
                               double map_resolution,
                               MatrixXf transition_matrix)
    : num_samples_(num_samples),
      num_timesteps_(num_timesteps),
      cov_matrix_(cov_matrix),
      u_min_(u_min),
      u_max_(u_max),
      lambda_(lambda),
      dt_(dt),
      map_width_(map_width),
      map_height_(map_height),
      control_dimension_(control_dimension),
      map_resolution_(map_resolution),
      transition_matrix_(transition_matrix) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MPPIController constructor");
  // Initialize variables
  if (cov_matrix_.rows() != cov_matrix_.cols()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Covariance matrix is not square");
    return;
  }
  input_number = cov_matrix_.rows();
  current_pose_.resize(control_dimension_, 1);
  u_.resize(input_number, num_timesteps_);

  samples_.resize(num_samples_);
  for (uint32_t i = 0; i < num_samples_; i++) {
    samples_[i].v_.resize(num_timesteps_);
    samples_[i].x_.resize(num_timesteps_);
    for (uint32_t j = 0; j < num_timesteps_; j++) {
      samples_[i].v_[j].resize(input_number, 1);
      samples_[i].x_[j].resize(control_dimension_, 1);
    }
  }

  gaussian_sampling();
  // show input sample but only num1
  for (uint32_t i = 0; i < num_samples_; i++) {
    for (uint32_t j = 0; j < num_timesteps_; j++) {
      for (uint8_t k = 0; k < input_number; k++) {
        std::cout << samples_[i].v_[k](j) << " ";
      }
      std::cout << std::endl;
    }
  }
}

MPPIController::~MPPIController() {}

// WIP: Only using diagonal elements of covariance matrix
void MPPIController::gaussian_sampling() {
  std::random_device rd;
  std::mt19937 gen(rd());

  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "GGGGGGGGGG");
  for (uint32_t i = 0; i < num_samples_; i++) {
    for (uint32_t j = 0; j < num_timesteps_; j++) {
      for (uint8_t k = 0; k < input_number; k++) {
        std::normal_distribution<double> dist(prev_u_[j](k, 0),
                                              cov_matrix_(k, k));
        samples_[i].v_[j](k, 0) = dist(gen);
        if (samples_[i].v_[j](k, 0) < u_min_[k]) {
          samples_[i].v_[j](k, 0) = u_min_[k];
        } else if (samples_[i].v_[j](k, 0) > u_max_[k]) {
          samples_[i].v_[j](k, 0) = u_max_[k];
        }
      }
    }
  }
}

void MPPIController::calc_predicted_state() {
  for (uint32_t i = 0; i < num_samples_; i++) {
    for (uint32_t j = 0; j < num_timesteps_; j++) {
      //   if (j == 0) {
      //     x_[i][0] = transition_matrix_ * current_pose_;
      //   } else {
      //     x_[i][j] = transition_matrix_ * x_[i][j - 1] + v_[i].
      //   }
    }
  }
}