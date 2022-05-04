#ifndef _ICM20948CFG_hpp
#define _ICM20948CFG_hpp

#include <cstdint>
// Default = +/- 4g. Valid ranges: 2, 4, 8, 16
enum AccelerometerFSR {
    AccelFSR2g = 2,
    AccelFSR4g = 4,  // default
    AccelFSR8g = 8,
    AccelFSR16g = 16,
};

// Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000
enum GyroscopeFSR {
    GyroFSR250dps = 250,
    GyroFSR500dps = 500,
    GyroFSR1000dps = 1000,
    GyroFSR2000dps = 2000,  // default
};

static constexpr double GyroScaleFactor(int32_t fsr) {
    switch (fsr) {
    case GyroFSR250dps: return 131.0;
    case GyroFSR500dps: return 65.5;
    case GyroFSR1000dps: return 32.8;
    case GyroFSR2000dps: return 16.4;
    default: return 0;
    }
}

static constexpr int32_t cfg_acc_fsr = AccelFSR4g;
static constexpr int32_t cfg_gyr_fsr = GyroFSR2000dps;
static constexpr double gyro_scale_factor = GyroScaleFactor(cfg_gyr_fsr);

// clang-format off
// 功能启用
static constexpr bool use_uncal_mag        = 0; // 未校准磁场
static constexpr bool use_uncal_gyro       = 0; // 未校准陀螺仪
static constexpr bool use_accel            = 0; // 加速度计
static constexpr bool use_gyro             = 1; // 陀螺仪
static constexpr bool use_mag              = 0; // 磁场
static constexpr bool use_gravity          = 0; // 重力 (基于6DOF位姿)
static constexpr bool use_linear_accel     = 0; // 线加速度 (基于6DOF位姿和加速度计)
static constexpr bool use_rpy_orientation  = 0; // 9DOF RPY (基于9DOF位姿)
static constexpr bool use_6DOF_orientation = 1; // 6DOF位姿
static constexpr bool use_9DOF_orientation = 1; // 9DOF位姿
static constexpr bool use_mag_orientation  = 0; // 地磁位姿

// 周期/ms
static constexpr uint32_t period_uncal_mag        = 100; // 未校准磁场 1~255Hz
static constexpr uint32_t period_uncal_gyro       = 100; // 未校准陀螺仪 1~255Hz
static constexpr uint32_t period_accel            = 100; // 加速度计 1~225Hz
static constexpr uint32_t period_gyro             = 1 ; // 陀螺仪 1~225Hz
static constexpr uint32_t period_mag              = 100; // 磁场 1~70Hz
static constexpr uint32_t period_gravity          = 20 ; // 重力 50~255Hz (基于6DOF位姿)
static constexpr uint32_t period_linear_accel     = 20 ; // 线加速度 50~255Hz (基于6DOF位姿和加速度计)
static constexpr uint32_t period_rpy_orientation  = 20 ; // 9DOF RPY 50~255Hz (基于9DOF位姿)
static constexpr uint32_t period_6DOF_orientation = 1 ; // 6DOF位姿 50~255Hz
static constexpr uint32_t period_9DOF_orientation = 1 ; // 9DOF位姿 50~255Hz
static constexpr uint32_t period_mag_orientation  = 20 ; // 地磁位姿 1~255Hz
// clang-format on

#endif  // _ICM20948CFG_hpp