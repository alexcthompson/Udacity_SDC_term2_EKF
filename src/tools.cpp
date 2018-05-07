#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
    return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    VectorXd res = estimations[i] - ground_truth[i];
      rmse = rmse.array() +  res.array() * res.array();
  }

  rmse = rmse.array() / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // convenience
  float denom_rho = sqrt(px * px + py * py);
  float denom_phi = px * px + py * py;
  float denom_rhoprime = denom_rho;

  // check division by zero
  if(denom_rho == 0){
      std::cout << "Division by zero!" << std::endl;
      return Hj;
  }

  // Set up zeros
  Hj(0, 2) = 0;
  Hj(0, 3) = 0;
  Hj(1, 2) = 0;
  Hj(1, 3) = 0;

  // compute the Jacobian matrix
  // rho
  Hj(0, 0) = px / denom_rho;
  Hj(0, 1) = py / denom_rho;
  
  // phi
  Hj(1, 0) = -py / denom_phi;
  Hj(1, 1) = px / denom_phi;
  
  // rhoprime
  Hj(2, 0) = py * (vx * py - vy * px) / pow(denom_rhoprime, 3);
  Hj(2, 1) = px * (vy * px - vx * py) / pow(denom_rhoprime, 3);
  Hj(2, 2) = px / denom_rhoprime;
  Hj(2, 3) = py / denom_rhoprime;

  return Hj;
}
