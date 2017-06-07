#include "FusionEKF.h"
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor
 */
FusionEKF::FusionEKF() 
{
  pEKF_ = std::make_shared<KalmanFilterExtended>();
}

/**
* Destructor
*/
FusionEKF::~FusionEKF() 
{
  ;
}



Eigen::VectorXd FusionEKF::GetState()
{
  return pEKF_->GetState();
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  // Check the measurement type is from a valid source.
  assert((measurement_pack.sensor_type_ == MeasurementPackage::RADAR) ||
         (measurement_pack.sensor_type_ == MeasurementPackage::LASER));

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!(pEKF_->IsInitialized()))
  {
    VectorXd initial_state(4);

    // first measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      // Used for positional calculation
      float ro     = measurement_pack.raw_measurements_[0];
      // Used for velocity calculation
      float ro_dot = measurement_pack.raw_measurements_[2];
      // Angle for polar coordinates
      float theta  = measurement_pack.raw_measurements_[1];

      float position_x = ro*cos(theta);
      float position_y = ro*sin(theta);
      float velocity_x = ro_dot*cos(theta);
      float velocity_y = ro_dot*sin(theta);

      initial_state << position_x, position_y, velocity_x, velocity_y;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      /**
      Initialize state.
      */
      float position_x  = measurement_pack.raw_measurements_[0];
      float position_y  = measurement_pack.raw_measurements_[1];

      initial_state << position_x, position_y, 0, 0;
    }

    pEKF_->Feed(initial_state, measurement_pack.timestamp_);
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  pEKF_->Predict(measurement_pack.timestamp_);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // Radar updates
    pEKF_->Update(measurement_pack.raw_measurements_);
  } 
  else
  {
    // Laser updates
    // Cast to the base kalman filter class first
    ((KalmanFilter *)(pEKF_.get()))->Update(measurement_pack.raw_measurements_);
  }
}
