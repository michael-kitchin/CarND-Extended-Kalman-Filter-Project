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
    TODO:
      * Finish initializing the FusionEKF.
      * Set the process and measurement noises
    */
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() = default;

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
        ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_
            == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            double rhoInM = measurement_pack.raw_measurements_[0]; // range
            double phiInRad = measurement_pack.raw_measurements_[1]; // bearing
            double rhoDot = measurement_pack.raw_measurements_[2]; // velocity of rho
            double xLoc = rhoInM * cos(phiInRad);

            if (xLoc < 0.0001) {
                xLoc = 0.0001;
            }

            double yLoc = rhoInM * sin(phiInRad);
            if (yLoc < 0.0001) {
                yLoc = 0.0001;
            }

            double xVel = rhoDot * cos(phiInRad);
            double yVel = rhoDot * sin(phiInRad);
            ekf_.x_ << xLoc, yLoc,
                    xVel, yVel;
        } else if (measurement_pack.sensor_type_
                   == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            ekf_.x_ << measurement_pack.raw_measurements_[0],
                    measurement_pack.raw_measurements_[1],
                    0, 0;
        }

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
    double diffTime = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    // update ST matrix
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, diffTime, 0,
            0, 1, 0, diffTime,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // build noise covariance matrix
    double noiseAx = 9.0;
    double noiseAy = 9.0;

    double diffTime2 = diffTime * diffTime; //dt^2
    double difffTime3 = diffTime2 * diffTime; //dt^3
    double diffTime4 = difffTime3 * diffTime; //dt^4
    double diffTime4d4 = diffTime4 / 4; //dt^4/4
    double diffTime3d2 = difffTime3 / 2; //dt^3/2

    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << diffTime4d4 * noiseAx, 0, diffTime3d2 * noiseAx, 0,
            0, diffTime4d4 * noiseAy, 0, diffTime3d2 * noiseAy,
            diffTime3d2 * noiseAx, 0, diffTime2 * noiseAx, 0,
            0, diffTime3d2 * noiseAy, 0, diffTime2 * noiseAy;

    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_
        == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);

    } else {
        // Laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
