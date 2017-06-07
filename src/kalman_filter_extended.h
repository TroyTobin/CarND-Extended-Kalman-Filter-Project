#ifndef KALMAN_FILTER_EXTENDED_H_
#define KALMAN_FILTER_EXTENDED_H_

#include "kalman_filter.h"

// Extended Kalman filter derived class
class KalmanFilterExtended: public KalmanFilter {
public:

  /**
   * Constructor
   */
  KalmanFilterExtended(); 

  /**
   * Destructor
   */
  virtual ~KalmanFilterExtended();

  /**
   * @brief Update the state by using Extended Kalman Filter equations
   *
   * @param z The new measurement
   */
  void Update(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_EXTENDED_H_ */
