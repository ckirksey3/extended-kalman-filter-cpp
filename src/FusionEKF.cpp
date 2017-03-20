#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

  //laser measurement covariance
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //laser measurement covariance
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

  tools = Tools();
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

    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //Convert radar from polar to cartesian coordinates and initialize state.
      double ro = measurement_pack.raw_measurements_[0];
      double theta = measurement_pack.raw_measurements_[1];
      double ro_dot = measurement_pack.raw_measurements_[2];

      double x = ro * std::cos(theta);
      double y = ro * std::sin(theta);

      VectorXd cartesian_measurement = VectorXd(4);
      cartesian_measurement << x, y, 0, 0;

      ekf_.x_ = cartesian_measurement;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0],
          measurement_pack.raw_measurements_[1],
          0,
          0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Modify the F matrix so that the time is integrated
  ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // Set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  (pow(dt, 4)/4)*noise_ax, 0, (pow(dt, 3)/2)*noise_ax, 0,
          0, (pow(dt, 4)/4)*noise_ay, 0, (pow(dt, 3)/2)*noise_ay,
          (pow(dt, 3)/2)*noise_ax, 0, (pow(dt, 2))*noise_ax, 0,
          0, (pow(dt, 3)/2)*noise_ay, 0, (pow(dt, 2))*noise_ay;

  // Call the Kalman Filter predict() function
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    VectorXd z(2);
    z << measurement_pack.raw_measurements_[0],
        measurement_pack.raw_measurements_[1];
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

#pragma clang diagnostic pop