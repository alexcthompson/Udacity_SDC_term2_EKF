#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  cout << "starting2" << endl;
  previous_timestamp_ = 0;

  // initializing matrices
  // R_laser_ = MatrixXd(2, 2);
  // R_radar_ = MatrixXd(3, 3);
  // H_laser_ = MatrixXd(2, 4);
  // Hj_ = MatrixXd(3, 4);

  // //measurement covariance matrix - laser
  // R_laser_ << 0.0225, 0,
  //       0, 0.0225;EKF

  // //measurement covariance matrix - radar
  // R_radar_ << 0.09, 0, 0,
  //       0, 0.0009, 0,
  //       0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);
      ekf_.x_(0) = cos(theta) * ro;
      ekf_.x_(1) = sin(theta) * ro;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;


    ekf_.Q_ = MatrixXd(4, 4);

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  cout << "Sensor type: " << measurement_pack.sensor_type_ << endl;
  cout << "Previous timestamp (ms) = " << previous_timestamp_ / 1000 << endl;
  cout << "Current timestamp (ms)  = " << measurement_pack.timestamp_ / 1000 << endl;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.0e6;
  cout << "∆t = " << dt << endl;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << endl << ekf_.x_ << endl << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
