#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
    VectorXd rmse(4);
    rmse << 0.0, 0.0, 0.0, 0.0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size())
        return rmse;

    // TODO: accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
        VectorXd e_g = estimations[i] - ground_truth[i];
        rmse += VectorXd(e_g.array() * e_g.array());
    }
    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();

    // return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    MatrixXd Hj(3, 4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // TODO: YOUR CODE HERE

    // check division by zero
    if (px == 0 && py == 0)
        return Hj;
    // compute the Jacobian matrix
    float x2_y2 = px * px + py * py;

    float r0_c0 = px / sqrt(x2_y2);
    float r0_c1 = py / sqrt(x2_y2);

    float r1_c0 = -py / x2_y2;
    float r1_c1 = px / x2_y2;

    float r2_c0 = py * (vx * py - vy * px) / pow(x2_y2, 3 / 2);
    float r2_c1 = px * (vy * px - vx * py) / pow(x2_y2, 3 / 2);
    float r2_c2 = r0_c0;
    float r2_c3 = r0_c1;

    Hj << r0_c0, r0_c1, 0, 0, r1_c0, r1_c1, 0, 0, r2_c0, r2_c1, r2_c2, r2_c3;
    return Hj;
}
