#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"


// Kalman filter base class.
class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * @brief Get the current state of the kalman filter
   *
   * @return The Kalman Filter State
   */
  Eigen::VectorXd GetState();


  /**
   * @brief Determine if the kalman filter is initialized
   *
   * @return True when initialized, False when not initialized
   */
  bool IsInitialized();

  /**
   * @brief Feed the kalman filter with the first measurement
   *
   * @param z The first measurement
   * @param timestamp The time of the first measurement
   */
  void Feed(const Eigen::VectorXd &z,
            long long timestamp);

  /**
   * @brief Predict the state and the state covariance using the process model.
   *
   * @param timestamp The offset of time to perform the state prediction
   */
  void Predict(long long timestamp);

  /**
   * @brief Update the state by using Kalman Filter equations
   *
   * @param z The new measurement
   */
  void Update(const Eigen::VectorXd &z);

private:  

  const float noise_ax = 9;
  const float noise_ay = 9;
  
  // check if the kalman filter has
  // been primed with the first measurement
  bool is_initialized_;

  // previous measurement timestamp
  long long previous_timestamp_;


protected:
  // tool object used to compute Jacobian and RMSE
  Tools tools;

};

#endif /* KALMAN_FILTER_H_ */
