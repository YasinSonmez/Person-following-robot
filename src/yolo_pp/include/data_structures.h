#ifndef KALMAN_FILTER_DATA_STRUCTURES_H_
#define KALMAN_FILTER_DATA_STRUCTURES_H_

#include "eigen3/Eigen/Dense"

enum KalmanState { X = 0, Y = 1, VX=2, VY=3 };
const unsigned int kNumStates = 4;
const double kMaxPredictTime = 30.0;

#endif // KALMAN_FILTER_DATA_STRUCTURES_H_
