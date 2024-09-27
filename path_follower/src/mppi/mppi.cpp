#include "mppi.hpp"

#include <random>
#include <rclcpp/rclcpp.hpp>

MPPIController::MPPIController(uint32_t num_samples, uint32_t num_timesteps,
                               MatrixXf cov_matrix, std::vector<double> u_min,
                               std::vector<double> u_max, double lambda,
                               double dt, uint16_t map_width,
                               uint16_t map_height,
                               uint8_t dimension_to_control,
                               uint8_t input_dimension, double map_resolution)
    : num_samples_(num_samples),
      num_timesteps_(num_timesteps),
      cov_matrix_(cov_matrix),
      u_min_(u_min),
      u_max_(u_max),
      lambda_(lambda),
      dt_(dt),
      map_width_(map_width),
      map_height_(map_height),
      pose_dimension_(dimension_to_control),
      input_dimension_(input_dimension),
      map_resolution_(map_resolution) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MPPIController constructor");
  // Initialize variables
  if (cov_matrix_.rows() != cov_matrix_.cols()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Covariance matrix is not square");
    return;
  }
  input_number = cov_matrix_.rows();
  current_pose_.resize(pose_dimension_, 1);
  u_.resize(num_timesteps_);
  for (uint32_t i = 0; i < num_timesteps_; i++) {
    u_[i].resize(input_number, 1);
  }
  prev_u_.resize(num_timesteps_);
  for (uint32_t i = 0; i < num_timesteps_; i++) {
    prev_u_[i].resize(input_number, 1);
  }
  samples_.resize(num_samples_);
  for (uint32_t i = 0; i < num_samples_; i++) {
    samples_[i].v_.resize(num_timesteps_);
    samples_[i].x_.resize(num_timesteps_);
    for (uint32_t j = 0; j < num_timesteps_; j++) {
      samples_[i].v_[j].resize(input_number, 1);
      samples_[i].x_[j].resize(pose_dimension_, 1);
    }
  }

  // initializing input
  for (uint32_t i = 0; i < num_timesteps_; i++) {
    for (uint8_t j = 0; j < input_number; j++) {
      prev_u_[i](j, 0) = 0.0;
    }
  }

  gaussian_sampling(prev_u_);
  calc_predicted_state();

  // show x samples
  for (uint32_t i = 0; i < num_samples_; i++) {
    for (uint32_t j = 0; j < num_timesteps_; j++) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x[%d][%d]: %f, %f, %f", i, j,
                  samples_[i].x_[j](0, 0), samples_[i].x_[j](1, 0),
                  samples_[i].x_[j](2, 0));
    }
  }
}

MPPIController::~MPPIController() {}

// WIP: Only using diagonal elements of covariance matrix
void MPPIController::gaussian_sampling(const std::vector<MatrixXf>& u) {
  std::random_device rd;
  std::mt19937 gen(rd());

  for (uint32_t i = 0; i < num_samples_; i++) {
    for (uint32_t j = 0; j < num_timesteps_; j++) {
      for (uint8_t k = 0; k < input_number; k++) {
        if (j == 0) {
          std::normal_distribution<double> dist(u[j](k, 0), cov_matrix_(k, k));
          samples_[i].v_[j](k, 0) = dist(gen);
        } else {
          std::normal_distribution<double> dist(samples_[i].v_[j - 1](k, 0),
                                                cov_matrix_(k, k));
          samples_[i].v_[j](k, 0) = dist(gen);
        }
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
      if (j == 0) {
        samples_[i].x_[j] = current_pose_;
      } else {
        samples_[i].x_[j] =
            diff_robot_transition_matrix(samples_[i].x_[j - 1]) *
                samples_[i].v_[j - 1] * dt_ +
            samples_[i].x_[j - 1];
      }
    }
  }
}

MatrixXf MPPIController::diff_robot_transition_matrix(const MatrixXf& x) {
  MatrixXf transition_matrix =
      MatrixXf::Zero(pose_dimension_, input_dimension_);
  transition_matrix(0, 0) = cos(x(2, 0));
  transition_matrix(0, 1) = -sin(x(2, 0));
  transition_matrix(1, 0) = sin(x(2, 0));
  transition_matrix(1, 1) = cos(x(2, 0));
  transition_matrix(2, 1) = 1.0;
  return transition_matrix;
}

double MPPIController::calc_cost(const singleSample& sample) {
  double cost = 0.0;
  



  return cost;
}
