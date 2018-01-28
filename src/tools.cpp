#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    return rmse;
  }

  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd residuals = estimations[i] - ground_truth[i];
    residuals = residuals.array() * residuals.array();
    rmse += residuals;
  }

  //calculate the mean
  rmse /= estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  if ((px == 0) && (py == 0)) {
    return Hj;
  }

  //compute the Jacobian matrix
  float sum_sq = px*px + py*py;
  float sum_sq_2 = sqrt(sum_sq);
  float sum_sq3_2 = sum_sq * sum_sq_2;

  Hj << px / sum_sq_2, py / sum_sq_2, 0, 0,
        -py / sum_sq, px / sum_sq, 0, 0,
        py*(vx*py - vy*px) / sum_sq3_2, px*(vy*px - vx*py) / sum_sq3_2, px / sum_sq_2, py / sum_sq_2;

  return Hj;
}
