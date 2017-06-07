#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter()
{
  is_initialized_ = false;

  x_ = VectorXd(4);
  P_ = MatrixXd::Identity(4, 4);
  F_ = MatrixXd::Identity(4, 4);
  Q_ = MatrixXd::Identity(4, 4);

  x_ << 1, 1, 1, 1;
}

KalmanFilter::~KalmanFilter()
{
  ;
}

Eigen::VectorXd KalmanFilter::GetState()
{
  return x_;
}

/**
 * @brief Determine if the kalman filter is initialized
 *
 * @return True when initialized, False when not initialized
 */
bool KalmanFilter::IsInitialized()
{
  return is_initialized_;
}

/**
 * @brief Feed the kalman filter with the first measurement
 *
 * @param z The first measurement
 * @param timestamp The time of the first measurement
 */
void KalmanFilter::Feed(const Eigen::VectorXd &z,
                        long long timestamp)
{
  x_ = z;
  is_initialized_ = true;
  previous_timestamp_ = timestamp;
}

/**
 * @brief Predict the state and the state covariance using the process model.
 *        Adapted from the Udacity lecture "Laser Measurements Part 4"
 *
 * @param timestamp The offset of time to perform the state prediction
 */
void KalmanFilter::Predict(long long timestamp)
{
  float predict_time_step, predict_time_step_2, predict_time_step_3, predict_time_step_4;
  float noise_ax_2, noise_ay_2;
  
  MatrixXd F_transpose;

  // Calculate the time offset for the prediction
  predict_time_step =  ((float)(timestamp - previous_timestamp_))/1e6;
  previous_timestamp_ = timestamp;

  // Set the state transistion matrix
  F_ << 1, 0, predict_time_step, 0,
        0, 1, 0,                 predict_time_step,
        0, 0, 1,                 0,
        0, 0, 0,                 1;

  
  predict_time_step_2 = predict_time_step*predict_time_step;
  predict_time_step_3 = predict_time_step_2*predict_time_step;
  predict_time_step_4 = predict_time_step_3*predict_time_step;

  noise_ax_2 = noise_ax*noise_ax;
  noise_ay_2 = noise_ay*noise_ay;

  // Set the process covariance matrix
  Q_ << (predict_time_step_4*noise_ax_2)/4, 0, (predict_time_step_3*noise_ax_2)/2, 0,
        0, (predict_time_step_4*noise_ay_2)/4, 0, (predict_time_step_3*noise_ay_2)/2,
        (predict_time_step_3*noise_ax_2)/2, 0, (predict_time_step_2*noise_ax_2), 0,
        0, (predict_time_step_3*noise_ay_2)/2, 0, (predict_time_step_2*noise_ay_2);

  // Perform the kalman filter prediction
  x_ = F_ * x_;
  F_transpose = F_.transpose();
  P_ = F_ * P_ * F_transpose + Q_;
}


/**
 * @brief Update the state by using Kalman Filter equations
 *        Adapted from the Udacity lecture "Kalman Filter Equations in C++ Part 2"
 * 
 * @param z The new measurement
 */
void KalmanFilter::Update(const VectorXd &z)
{
  // measurement covariance matrix
  Eigen::MatrixXd R_ = MatrixXd(2, 2);
  
  // measurement matrix
  Eigen::MatrixXd H_ = MatrixXd(2, 4);

  VectorXd y;
  MatrixXd H_transpose, S, S_inverse, K, I;

  // Sanity check the z input
  if (z.size() != 2)
  {
    cerr << "KalmanFilter:Update - Invalid input size ";
    cerr << z.size() << " != " << 2 << endl;
    goto Error; 
  }

  // Set the measurement covariance matrix
  R_ << 0.04, 0,
        0,      0.04;

  // Set the measurement matrix for the "base" update function
  // This indicates that the measurements are x,y positions, but no velocities
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

  // Update the kalman filter state
  y = z - H_ * x_;
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
