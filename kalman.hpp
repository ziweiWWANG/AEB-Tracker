// This file is licensed under the Apache License, Version 2.0,
// with additional restrictions under the Commons Clause.
// See the LICENSE file for more details.

#include <eigen3/Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <iomanip>
#pragma once

class KalmanFilter
{

public:
  /**
   * Create a Kalman filter with the specified matrices.
   *   F - System dynamics matrix
   *   C - Output matrix
   *   Q - Process noise covariance
   *   R - Measurement noise covariance
   *   P - Estimate error covariance
   */
  KalmanFilter(double dt, const Eigen::MatrixXd &F, const Eigen::MatrixXd &C,
               const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, const Eigen::MatrixXd &P,
               const Eigen::MatrixXd &A, const Eigen::MatrixXd &x0, int n_state, int n_target,
               int ring_buffer_len);

  /**
   * Create a blank estimator.
   */
  KalmanFilter();

  /**
   * Initialize the filter with initial states as zero.
   */
  void init();

  /**
   * Initialize the filter with a guess for initial states.
   */
  void init(double t0, std::string input_folder_nam);

  /**
   * Update the estimated state based on measured values. The
   * time step is assumed to remain constant.
   */
  void update(const Eigen::Vector3d &y, int id, int p);

  void update_start_point(const Eigen::MatrixXd &x0) { x_hat = x0; }

  void update_gyro(const Eigen::MatrixXd vx_gyro_update, const Eigen::MatrixXd vy_gyro_update)
  {
    x_hat.block(0, 0, n_target, 1) = x_hat.block(0, 0, n_target, 1) + vx_gyro_update;
    x_hat.block(0, 1, n_target, 1) = x_hat.block(0, 1, n_target, 1) + vy_gyro_update;
  }

  void update_matrix(const Eigen::MatrixXd &Fx, const Eigen::MatrixXd &Cx,
                     const Eigen::MatrixXd &Qx, const Eigen::MatrixXd &Rx, const Eigen::MatrixXd &Px,
                     const Eigen::MatrixXd &Ax, const Eigen::MatrixXd &x0x)
  {
    F = Fx;
    C = Cx;
    Q = Qx;
    R = Rx;
    P = Px;
    A = Ax;
    x_hat = x0x;
  }

  /**
   * Return the current state and time.
   */
  Eigen::MatrixXd state() { return x_hat; };
  double time() { return t; };
  double P_lambda() { return P(4, 4); };
  // P, TODO: update i to P
  Eigen::MatrixXd P_x() { return P; };

private:
  int n_state, n_target;
  // Matrices for computation
  Eigen::MatrixXd F, C, Q, R, P, K, P0, A;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::MatrixXd x_hat;

  std::vector<double> ts_last;

  Eigen::Vector2d e_tilda, y_hat, y_hat_sum, y_hat_buffer, e_tilda_buffer;
  Eigen::MatrixXd S, sqrtMinv;
  double y_square_sum, theta_buf;
  Eigen::Matrix<double, 3, 1> y_hat_fuse, y_true;
  Eigen::MatrixXd I2 = Eigen::MatrixXd::Identity(2, 2);
  std::ofstream track_output_txt_;

  Eigen::Matrix2d rotation_m, rotation_m_buf, diag_m, Omiga, A_rotation, C_lambda;
  Eigen::Matrix<double, 2, 1> C_lambda_diag;

  Eigen::Matrix<double, 1, 2> y_lambda, y_lambda_sum;
  Eigen::Matrix<double, 2, 1> temp_diag;
  Eigen::Matrix2d temp;
  int ring_buffer_len;
  double vx_gyro = 0;
  double vy_gyro = 0;
};
