// This file is licensed under the Apache License, Version 2.0,
// with additional restrictions under the Commons Clause.
// See the LICENSE file for more details.

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "kalman.hpp"

typedef boost::circular_buffer<Eigen::MatrixXd> CircularBuffer;
std::vector<boost::circular_buffer<std::tuple<double, double, double, double>>> ring_buffer_e;
std::vector<boost::circular_buffer<std::pair<double, double>>> ring_buffer_x;
std::vector<boost::circular_buffer<double>> ring_buffer_theta;
std::vector<boost::circular_buffer<std::pair<double, double>>> ring_buffer_delta;

KalmanFilter::KalmanFilter(double dt, const Eigen::MatrixXd &F, const Eigen::MatrixXd &C,
                           const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                           const Eigen::MatrixXd &P, const Eigen::MatrixXd &A,
                           const Eigen::MatrixXd &x0, int n_state, int n_target,
                           int ring_buffer_len)
    : F(F), C(C), Q(Q), R(R), P0(P), A(A), dt(dt), initialized(false), I(F.cols(), F.cols()),
      x_hat(x0), n_state(n_state), n_target(n_target), ring_buffer_len(ring_buffer_len)
{
  I.setIdentity();
  x_hat = x0;
  for (int i = 0; i < n_target; ++i)
  {
    ring_buffer_e.emplace_back(ring_buffer_len);
    ring_buffer_x.emplace_back(ring_buffer_len);
    ring_buffer_theta.emplace_back(ring_buffer_len);
    ring_buffer_delta.emplace_back(ring_buffer_len);
  }
}
KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, std::string input_folder_name)
{
  P = P0;
  this->t0 = t0;
  initialized = true;
  Omiga << 0, -1, 1, 0;
  y_true << 0, 0, 2 * ring_buffer_len;

  // Important for re-init different targets
  ts_last.clear();
  ring_buffer_e.clear();
  ring_buffer_x.clear();
  ring_buffer_theta.clear();
  ring_buffer_delta.clear();
  for (int i = 0; i < n_target; i++)
  {
    ts_last.push_back(0);
    ring_buffer_e.emplace_back(ring_buffer_len);
    ring_buffer_x.emplace_back(ring_buffer_len);
    ring_buffer_theta.emplace_back(ring_buffer_len);
    ring_buffer_delta.emplace_back(ring_buffer_len);
  }
  std::cout << "kf init x_hat: " << x_hat << std::endl;
}

void KalmanFilter::init()
{
  P = P0;
  t0 = 0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::Vector3d &e, int id, int p)
{

  if (!initialized)
    throw std::runtime_error("Filter is not initialized!");

  if (ts_last[id] <= 0)
  {
    dt = 0; // First event triggered at this pixel
  }
  else
  {
    dt = (e(2) - ts_last[id]);
  }

  ts_last[id] = e(2);
  F(id * n_state, 2) = dt;
  F(id * n_state + 1, 3) = dt;
  F(id * n_state + 6, 7) = dt;
  x_hat.block(id, 0, 1, n_state) =
      (F.block(id * n_state, 0, n_state, n_state) * x_hat.block(id, 0, 1, n_state).transpose())
          .transpose();

  P.block(id * n_state, 0, n_state, n_state) =
      F.block(id * n_state, 0, n_state, n_state) * P.block(id * n_state, 0, n_state, n_state) *
          F.block(id * n_state, 0, n_state, n_state).transpose() +
      Q * dt;
  rotation_m << cos(x_hat(id, 6)), -sin(x_hat(id, 6)), sin(x_hat(id, 6)), cos(x_hat(id, 6));

  // A is 1 on lambda
  A << 1 / x_hat(id, 4), 0, 0, 1 / x_hat(id, 5);
  A_rotation = rotation_m * A * rotation_m.transpose();

  if (p <= 0)
  {
    e_tilda << e(0) + x_hat(id, 8) - x_hat(id, 0), e(1) + x_hat(id, 9) - x_hat(id, 1);
    ring_buffer_delta[id].push_back({x_hat(id, 8), x_hat(id, 9)});
  }
  else
  {
    e_tilda << e(0) - x_hat(id, 8) - x_hat(id, 0), e(1) - x_hat(id, 9) - x_hat(id, 1);
    ring_buffer_delta[id].push_back({-x_hat(id, 8), -x_hat(id, 9)});
  }

  y_hat = A_rotation * e_tilda;
  y_hat_sum << 0, 0;
  y_square_sum = 0;
  C.block(0, 0, 2, 2) << -A_rotation;
  C.block(0, 2, 2, 2) << 0, 0, 0, 0;
  C_lambda_diag = A * A * rotation_m.transpose() * e_tilda;
  C_lambda << C_lambda_diag(0), 0, 0, C_lambda_diag(1);
  C.block(0, 4, 2, 2) << -rotation_m * C_lambda;
  C.block(0, 6, 2, 1) << rotation_m * (Omiga * A - A * Omiga) * rotation_m.transpose() * e_tilda;
  C.block(0, 7, 2, 1) << 0, 0;

  if (p <= 0)
  {
    C.block(0, 8, 2, 2) << A_rotation;
  }
  else
  {
    C.block(0, 8, 2, 2) << -A_rotation;
  }

  C.block(2, 0, 1, n_state) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  ring_buffer_e[id].push_back({e(0), e(1), e(2), p});
  ring_buffer_x[id].push_back({x_hat(id, 0), x_hat(id, 1)});
  ring_buffer_theta[id].push_back(x_hat(id, 6));

  if (ring_buffer_e[id].size() >= 2)
  {
    // t1, t2, t3, ..., buf_dt[last] is the latest for buffer e, x and m
    // here use t1, t2, t3, ..., buf_dt[last]-1
    y_lambda << 0, 0;
    y_lambda_sum << 0, 0;
    for (int i = (ring_buffer_e[id].size() - 2); i >= 0; --i)
    {
      // Use the same time states and events
      e_tilda_buffer << std::get<0>(ring_buffer_e[id][i]) + ring_buffer_delta[id][i].first - ring_buffer_x[id][i].first,
          std::get<1>(ring_buffer_e[id][i]) + ring_buffer_delta[id][i].second - ring_buffer_x[id][i].second;

      theta_buf = ring_buffer_theta[id][i];
      rotation_m_buf << cos(theta_buf), -sin(theta_buf), sin(theta_buf), cos(theta_buf);
      y_hat_buffer = rotation_m_buf * A * rotation_m_buf.transpose() * e_tilda_buffer;
      y_hat_sum += y_hat_buffer;
      y_square_sum += pow(y_hat_buffer(0), 2) + pow(y_hat_buffer(1), 2);

      temp_diag = A * A * rotation_m_buf.transpose() * e_tilda_buffer;
      temp << temp_diag(0), 0, 0, temp_diag(1);

      y_lambda = -2 * y_hat_buffer.transpose() * rotation_m_buf * temp;
      y_lambda_sum += y_lambda;
    }
    C.block(2, 4, 1, 2) += y_lambda_sum;
  }

  // Innovation covariance
  R(2, 2) = 4 * ring_buffer_e[id].size();
  S = C * P.block(id * n_state, 0, n_state, n_state) * C.transpose() + R;
  K = P.block(id * n_state, 0, n_state, n_state) * C.transpose() * S.inverse();
  y_hat_fuse << y_hat, y_square_sum;
  y_true.coeffRef(2, 0) = 2 * ring_buffer_e[id].size();
  x_hat.block(id, 0, 1, n_state) += (K * (y_true - y_hat_fuse)).transpose(); // y = {0, 0}
  P.block(id * n_state, 0, n_state, n_state) =
      (I - K * C) * P.block(id * n_state, 0, n_state, n_state);
}
