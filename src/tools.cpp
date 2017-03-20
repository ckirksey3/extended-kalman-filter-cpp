#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    if(estimations.size() < 1) {
        throw std::invalid_argument( "Error: Estimation vector size should not be zero");
    }
    else if(estimations.size() != ground_truth.size()) {
        throw std::invalid_argument( "Error: Estimation vector size should equal ground truth size");
    } else {
        //accumulate squared residuals
        for(int i=0; i < estimations.size(); ++i){
            VectorXd current_estimate = estimations[i];
            VectorXd cur_ground_truth = ground_truth[i];
            VectorXd residual = current_estimate - cur_ground_truth;
            for(int j=0; j<residual.size(); ++j) {
                residual[j] = residual[j] * residual[j];
            }
            rmse += residual;
        }

        //calculate the mean
        rmse /= estimations.size();

        //calculate the squared root
        for(int i=0; i<rmse.size(); ++i) {
            rmse[i] = sqrt(rmse[i]);
        }

        //return the result
        return rmse;
    }
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //check division by zero
    if(px == 0 || py == 0) {
        throw std::invalid_argument("Division by zero error calculation jacobian");
    }

    //compute the Jacobian matrix
    float px2 = px * px;
    float py2 = py * py;
    float px2Ppy2 = px2 + py2;
    float px2Ppy2Sq = sqrt(px2Ppy2);
    float bigOne = (vx*py - vy*px)/(pow(px2Ppy2, 3/2));

    Hj << px/px2Ppy2Sq, py/px2Ppy2Sq, 0, 0,
            -py/px2Ppy2, px/px2Ppy2, 0, 0,
            py*bigOne, px*bigOne, px/px2Ppy2Sq, py/px2Ppy2Sq;

    return Hj;
}

VectorXd Tools::CalculateNonlinearH(const VectorXd& x_state) {
    VectorXd h_non_linear(3,1);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //check division by zero
    if(px == 0) {
        throw std::invalid_argument("Division by zero error calculation nonlinear H");
    }

    //compute the polar coordinates
    float px2 = px * px;
    float py2 = py * py;
    float px2Ppy2 = px2 + py2;
    float px2Ppy2Sq = sqrt(px2Ppy2);
    float bigOne = (vx*px + vy*py)/px2Ppy2Sq;

    h_non_linear << px2Ppy2Sq,
        atan(py/px),
        bigOne;

    return h_non_linear;
}

// From: http://stackoverflow.com/a/29871193/1321129
/* change to `float/fmodf` or `long double/fmodl` or `int/%` as appropriate */
/* wrap x -> [0,max) */
double Tools::wrapMax(double x, double max)
{
  /* integer math: `(max + x % max) % max` */
  return fmod(max + fmod(x, max), max);
}

/* wrap x -> [min,max) */
double Tools::wrapMinMax(double x, double min, double max)
{
  return min + wrapMax(x - min, max - min);
}