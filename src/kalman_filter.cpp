#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
    //measurement matrix
  H_ = MatrixXd(2, 4);
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225,      0,
                   0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09,      0,      0,
                 0, 0.0009,      0,
                 0,      0,   0.09;

  noise_ax = 9;
  noise_ay = 9;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  // R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict(const float &dt) {
  /**
  TODO:
    * predict the state
  */
  // std::cout << "P before Predict: \n" << std::endl << P_ << std::endl;
  F_= MatrixXd(4, 4);
  F_ << 1, 0, dt,  0,
        0, 1,  0, dt,
        0, 0,  1,  0,
        0, 0,  1,  0;

  float dt2 = pow(dt, 2.0);
  float dt3 = pow(dt, 3.0);
  float dt4 = pow(dt, 4.0);

  Q_ << dt4 * noise_ax / 4.0,            0, dt3 * noise_ax / 2.0,            0,
                   0, dt4 * noise_ay / 4.0,            0, dt3 * noise_ay / 2.0,
        dt3 * noise_ax / 2.0,            0,       dt2 * noise_ax,            0,
                   0, dt3 * noise_ay / 2.0,            0,       dt2 * noise_ay;

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  std::cout << "y = " << std::endl << y << std::endl;
  MatrixXd S = H_ * P_ * H_.transpose() + R_laser_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  std::cout << "K = " << std::endl << K << std::endl;


  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
