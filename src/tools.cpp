#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() = default;

Tools::~Tools() = default;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    TODO:
      * Calculate the RMSE here.
    */
    VectorXd resultRmse(4);
    resultRmse << 0, 0, 0, 0;

    // pre-checks
    if (estimations.size() == 0) {
        cout << "Can't calculate RMSE (empty estimations vector)." << endl;
        return resultRmse;
    }

    if (ground_truth.size() == 0) {
        cout << "Can't calculate RMSE (empty ground truth vector)." << endl;
        return resultRmse;
    }

    unsigned long estSize = estimations.size();
    if (estSize != ground_truth.size()) {
        cout << "Can't calculate RMSE (same-size estimations/ground truth vector)." << endl;
        return resultRmse;
    }

    for (unsigned int ctr = 0;
         ctr < estSize;
         ++ctr) {
        VectorXd diffVec = estimations[ctr] - ground_truth[ctr];
        diffVec = diffVec.array() * diffVec.array();
        resultRmse += diffVec;
    }

    resultRmse = resultRmse / estSize;
    resultRmse = resultRmse.array().sqrt();
    return resultRmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    /**
    TODO:
      * Calculate a Jacobian here.
    */
    MatrixXd hJ(3, 4);

    // pre-check
    if (x_state.size() != 4) {
        cout << "Can't calculate Jacobian (invalid input vector size)." << endl;
        return hJ;
    }

    // extract state params
    double xLocP = x_state(0);
    double yLocP = x_state(1);
    double xVelP = x_state(2);
    double yVelP = x_state(3);

    // cache working terms
    double workC1 = (xLocP * xLocP) + (yLocP * yLocP);
    double workC2 = sqrt(workC1);
    double workC3 = (workC1 * workC2);

    // DBZ check (range check due to precision loss)
    if (fabs(workC1) < 0.0001) {
        cout << "Can't calculate Jacobian (divide by zero)." << endl;
        return hJ;
    }

    // Jacobian matrix
    hJ << (xLocP / workC2), (yLocP / workC2), 0, 0,
            -(yLocP / workC1), (xLocP / workC1), 0, 0,
            yLocP * (xVelP * yLocP - yVelP * xLocP)
            / workC3, xLocP * (xLocP * yVelP - yLocP * xVelP)
                      / workC3,
            xLocP / workC2,
            yLocP / workC2;

    return hJ;
}
