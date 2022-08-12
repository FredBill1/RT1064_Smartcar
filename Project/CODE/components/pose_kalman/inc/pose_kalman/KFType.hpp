#ifndef _pose_kalman_modelType_hpp
#define _pose_kalman_modelType_hpp

#include "pose_kalman/config.hpp"

#if defined(pose_kalman_use_squareroot) && pose_kalman_use_squareroot

#include "kalman/SquareRootBase.hpp"
#include "kalman/SquareRootExtendedKalmanFilter.hpp"

#define pose_kalman_KF Kalman::SquareRootExtendedKalmanFilter
#define pose_kalman_COVBASE Kalman::SquareRootBase

#else

#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/StandardBase.hpp"

#define pose_kalman_KF Kalman::ExtendedKalmanFilter
#define pose_kalman_COVBASE Kalman::StandardBase

#endif  // pose_kalman_use_squareroot

#endif  // _pose_kalman_modelType_hpp