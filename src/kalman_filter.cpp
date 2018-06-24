#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

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
    x_ = F_ * x_;
    MatrixXd fT = F_.transpose();
    P_ = F_ * P_ * fT + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Kalman Filter equations
    */
    VectorXd yVec = z - H_ * x_;
    UpdateFromY(yVec);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */
    double xLocP = x_(0);
    double yLocP = x_(1);
    double xVelP = x_(2);
    double yVelP = x_(3);

    double rhoP = sqrt(xLocP * xLocP + yLocP * yLocP);
    double thetaP = atan2(yLocP, xLocP);
    double rhoDot = (xLocP * xVelP + yLocP * yVelP) / rhoP;

    auto hP = VectorXd(3);
    hP << rhoP, thetaP, rhoDot;
    VectorXd yP = z - hP;

    while (yP(1) > M_PI || yP(1) < -M_PI) {
        if (yP(1) > M_PI) {
            yP(1) -= M_PI;
        } else {
            yP(1) += M_PI;
        }
    }
    UpdateFromY(yP);
}

void KalmanFilter::UpdateFromY(const VectorXd &y) {
    MatrixXd hT = H_.transpose();
    MatrixXd s = H_ * P_ * hT + R_;
    MatrixXd sI = s.inverse();
    MatrixXd k = P_ * hT * sI;
    x_ = x_ + (k * y);
    long xSize = x_.size();
    MatrixXd i = MatrixXd::Identity(xSize, xSize);
    P_ = (i - k * H_) * P_;
}
