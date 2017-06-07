#include "kalman_filter_extended.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


KalmanFilterExtended::KalmanFilterExtended(): KalmanFilter()
{
  ;
}

KalmanFilterExtended::~KalmanFilterExtended()
{
  ;
}

/**
 * @brief  Update the kalman filter for the Radar data
 *         Adapted from the Udacity lecture "EKF Algorithm Generalization"
 *
 */
void KalmanFilterExtended::Update(const VectorXd &z)
{
  float position_x, position_y,velocity_x,velocity_y;
  float c1, c2, c3, c4;

  // measurement covariance matrix
  Eigen::MatrixXd R_ = MatrixXd(3, 3);
  
  // measurement matrix
  Eigen::MatrixXd H_ = MatrixXd(3, 4);

  VectorXd y;
  MatrixXd H_transpose, S, S_inverse, K, I;
  VectorXd h_x(3);

  // Sanity check the z input
  if (z.size() != 3)
  {
    cerr << "KalmanFilterExtended:Update - Invalid input size ";
    cerr << z.size() << " != " << 3 << endl;
    goto Error; 
  }

  // Sanity check the angle provided from the radar signal.
  // If not in bounds ignore
  if (fabs(z[1]) > M_PI)
  {
    goto Error;
  }

  //measurement covariance matrix - radar
  R_ << 0.15, 0,      0,
        0,    0.0015, 0,
        0,    0,      0.15;

  // Use the Jacobian for the measurement matrix
  H_ = tools.CalculateJacobian(x_);

  // Extract the Position and Velocity state
  position_x = x_(0);
  position_y = x_(1);
  velocity_x = x_(2);
  velocity_y = x_(3);

  // Compute some values useful in the Extended Kalman Filter
  c1 = (position_x*position_x) + (position_y*position_y);
  c2 = sqrt(c1);
  c3 = atan2(position_y, position_x);
  c4 = (position_x*velocity_x) + (position_y*velocity_y);

  // Check division by zero
  if(fabs(c2) < numeric_limits<float>::epsilon())
  {
    cerr << "KalmanFilterExtended:Update - Error - Division by Zero" << endl;
    goto Error;
  }

  h_x << c2, c3, c4/c2;
  y = z - h_x;

  H_transpose = H_.transpose();
  S = H_ * P_ * H_transpose + R_;
  S_inverse = S.inverse();
  K =  P_ * H_transpose * S_inverse;
  I = MatrixXd::Identity(4, 4);

  //new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;

Error:
  ;
}
