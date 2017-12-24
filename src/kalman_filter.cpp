#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <math.h>

#define PI 3.14159265

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {

  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    //std::cout<<"x size is: "<<x_.size();
    //std::cout<<"F size is: "<<F_.size();
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

    VectorXd z_pred = H_ * x_;

    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size , x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);

    // If rho == 0, skip the update step to avoid dividing by zero.
    // This is crude but should be fairly robust on our data set.
    // if( px == 0. && py == 0. )
    //     return;

    VectorXd hofx(3);

    double rho = sqrt( px*px + py*py );
    if (rho < 0.000001) {
      rho = 0.000001;
    }
    double rho_d = (( px*vx + py*vy )/rho);

    hofx << rho, atan2( py, px ), rho_d;
    
    VectorXd y = z - hofx;
    
    while(y(1) > PI){
    y(1) -= (2*PI);
  }

  while(y(1) < -PI){
    y(1) += (2*PI);
  }

    MatrixXd Hjt = H_.transpose();
    MatrixXd S = H_*P_*Hjt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_*Hjt*Si;

    // Compute new state
    x_ = x_ + ( K*y );
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size , x_size);
    P_ = ( I - K * H_ ) * P_;




}
