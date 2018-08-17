//
// Created by hd on 4/19/18.
//

#ifndef PROJECT_KALMAN_H
#define PROJECT_KALMAN_H

#include <Eigen/Dense>

class KalmanFilter
{
public:
    explicit KalmanFilter() = default;

    KalmanFilter(unsigned int dim_in, unsigned int dim_out, unsigned int dim_state):
            input_d(dim_in), output_d(dim_out), state_d(dim_state) {

        state_m.setIdentity(state_d, state_d);
        input_m.setZero(state_d, input_d);
        output_m.setZero(output_d, state_d);

        gain_m.setIdentity(state_d, output_d);
        I.setIdentity(state_d, state_d);

        P.setIdentity(state_d, state_d);
        Q.setIdentity(state_d, state_d);
        R.setIdentity(output_d, output_d);
        S.setIdentity(output_d, output_d);

        input_sig.setZero(input_d);
        q_pred.setZero(state_d);
        q_est.setZero(state_d);
        measure_sig.setZero(output_d);
    }

    void predictState() {
        q_pred = state_m * q_est + input_m * input_sig;
        P = state_m * P * state_m.transpose() + Q;
    }

    void correctState() {
        S = output_m * P * output_m.transpose() + R;
        gain_m = P * output_m.transpose() * S.inverse();
        q_est = q_pred + gain_m * (measure_sig - output_m * q_pred);
        P = (I - gain_m * output_m) * P;
    }

    void updateState() {
        predictState();
        correctState();
    }

    // Dimensions:
    unsigned int input_d{};                   // Input
    unsigned int output_d{};                  // Output
    unsigned int state_d{};                   // State

    // System matrices:
    Eigen::MatrixXd state_m;                // State
    Eigen::MatrixXd input_m;                // Input
    Eigen::MatrixXd output_m;               // Output

    // Kalman gain matrix:
    Eigen::MatrixXd gain_m;

    // Identity matrix
    Eigen::MatrixXd I;

    // Covariance matrices:
    Eigen::MatrixXd P;                      // Estimate error
    Eigen::MatrixXd Q;                      // Process
    Eigen::MatrixXd R;                      // Measurement
    Eigen::MatrixXd S;                      // Innovation

    // Signals:
    Eigen::VectorXd input_sig;              // Input
    Eigen::VectorXd q_pred;                 // Predicted state
    Eigen::VectorXd q_est;                  // Estimated state
    Eigen::VectorXd measure_sig;            // Measurement
};
#endif //PROJECT_KALMAN_H
