#include "kalman_filter.h"

#include <iostream>

KalmanFilter::KalmanFilter() {
  init();
  is_initial_ = true;
}

void KalmanFilter::init(){
  (void)I_.setIdentity();
  (void)A_.setIdentity();
  (void)H_.setIdentity();
  H_(KalmanState::VX, KalmanState::VX) = 0.0;
  H_(KalmanState::VY, KalmanState::VY) = 0.0;
  (void)K_.setZero();

  P_ << 9999, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 9999, 0, 0, 0, 0, 9999;
  Q_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  R_ << 9, 0, 0, 0, 0, 9, 0, 0, 0, 0, 999, 0, 0, 0, 0, 999;

  (void)state_.setZero();
}
Eigen::Matrix<double,kNumStates,1> KalmanFilter::predict(double current_time) {

  auto dt = current_time - last_update_time_;
  if (is_initial_ || dt > kMaxPredictTime) {
    is_initial_ = false;
    init();
  }
  last_update_time_ = current_time;
  A_(KalmanState::X, KalmanState::VX) = dt;
  A_(KalmanState::Y, KalmanState::VY) = dt;
  state_ = A_ * state_;
  P_ = A_ * P_ * A_.transpose() + Q_;

  return state_;
}

Eigen::Matrix<double,kNumStates,1> KalmanFilter::update(const Eigen::Matrix<double,kNumStates,1> &measurement) {

        K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
        state_ += K_ * (measurement - H_ * state_);
        P_ = (I_ - K_ * H_) * P_;



  return state_;
}
