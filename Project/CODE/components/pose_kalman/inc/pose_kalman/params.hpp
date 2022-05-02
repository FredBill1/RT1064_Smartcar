#ifndef _pose_kalman_params_hpp
#define _pose_kalman_params_hpp

#include <cstdint>

#include "pose_kalman/config.hpp"
namespace pose_kalman {

constexpr uint64_t predict_period_us = 10000;
constexpr T predict_period_ms = predict_period_us * 1e-3;
constexpr T predict_period_s = predict_period_us * 1e-6;
constexpr T predict_freq = 1. / predict_period_s;

constexpr T sys_xy_sigma2 = 0.05;
constexpr T sys_yaw_sigma2 = 0.06;
constexpr T sys_v_xy_sigma2 = 0.025;
constexpr T sys_v_yaw_sigma2 = 0.02;

constexpr T odom_v_xy_sigma2 = 1e-2;
constexpr T odom_v_yaw_sigma2 = 1e-2;

constexpr T gyro_v_yaw_sigma2 = 1e-3;

}  // namespace pose_kalman

#endif  // _pose_kalman_params_hpp