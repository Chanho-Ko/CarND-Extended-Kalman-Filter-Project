#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  ekf_.P_ = MatrixXd(4,4);
  ekf_.F_ = MatrixXd(4,4);
  ekf_.Q_ = MatrixXd(4,4);
  
  ekf_.P_ << 100, 0, 0, 0,
            0, 100, 0, 0,
            0, 0, 100, 0,
            0, 0, 0, 100;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  long long current_timestamp_ = measurement_pack.timestamp_;
  long long dt_int = current_timestamp_- previous_timestamp_;
  previous_timestamp_ = current_timestamp_;

  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    // firt time step
    dt_int = 50000;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      ekf_.H_ = MatrixXd(3,4);
      ekf_.R_ = MatrixXd(3,3);
      float rho = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);
      //cout << "## HERE1 ## " << endl;
      ekf_.x_ << rho*cos(theta), rho*sin(theta), rho_dot*cos(theta), rho_dot*cos(theta);
      cout << "x: " << ekf_.x_ << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.H_ = MatrixXd(2,4);
      ekf_.R_ = MatrixXd(2,2);
      //cout << "## HERE2 ## " << endl;
      ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
      cout << "x: " << ekf_.x_ << endl;
    }
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    
  }
  
  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = dt_int*0.000001;
  float var_x = 9, var_y = 9;
  ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
  ekf_.Q_ << pow(dt,4)/4*var_x, 0, pow(dt,3)/2*var_x, 0,
            0, pow(dt,4)/4*var_y, 0, pow(dt,3)/2*var_y,
            pow(dt,3)/2*var_x, 0, dt*dt*var_x, 0,
            0, pow(dt,3)/2*var_y, 0, dt*dt*var_y;    
  ekf_.Predict();
  //cout << "####### After Prediction ######" << endl;   
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.H_ = MatrixXd(3,4);
    ekf_.R_ = MatrixXd(3,3);  
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    //cout << "####### Before RADAR Update ######" << endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else { 
    // TODO: Laser updates
    
    ekf_.H_ = MatrixXd(2,4);
    ekf_.R_ = MatrixXd(2,2);
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    //cout << "####### Before LIDAR Update ######" << endl;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  //out << "####### After Update ######" << endl;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
