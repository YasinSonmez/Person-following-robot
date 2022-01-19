#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "eigen3/Eigen/Dense"
#include "data_structures.h"

/// @brief This class contains kalman filter calculation for the state estimation
class KalmanFilter {
  public:
    explicit KalmanFilter();
    /// @brief Predicts state using Ax (no input)
    Eigen::Matrix<double,kNumStates,1> predict(double current_time);
    /// @brief Updates state based on new measurement
    /// @param measurement  New measurement
    /// @param source  Measurement source (model or imu)
    Eigen::Matrix<double,kNumStates,1> update(const Eigen::Matrix<double,kNumStates,1> &measurement);

  private:
    /// Matrices for computation
    Eigen::Matrix<double,kNumStates,kNumStates> A_, Q_, P_, K_;
    Eigen::Matrix<double,kNumStates,kNumStates> H_, R_;
    /// Current state of the kalman filter
    Eigen::Matrix<double,kNumStates,1> state_;

    /// Identity matrix with same dimensions as A
    Eigen::Matrix<double,kNumStates,kNumStates> I_;
    double last_update_time_;
    bool is_initial_;

    void init();
};

#endif // KALMAN_FILTER_H_
