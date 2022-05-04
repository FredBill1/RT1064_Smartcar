#include "ICM20948.hpp"
#include "ICM20948CFG.hpp"
//
#include <cmath>

#include "devices.hpp"
#include "pose_kalman/utils.hpp"

using namespace pose_kalman;

#define send_data(name, len, id)                                        \
    {                                                                   \
        static SerialIO::TxUtil<float, len, true> name##_tx(#name, id); \
        if (name##_tx.txFinished()) {                                   \
            name##_tx.setArr(data);                                     \
            wireless.send(name##_tx);                                   \
        }                                                               \
    }

static void imu_cb_uncal_mag(const float data[6], uint64_t timestamp_us) { send_data(mag_uncal, 6, 0); }
static void imu_cb_uncal_gyro(const float data[6], uint64_t timestamp_us) { send_data(gyro_uncal, 6, 1); }
static void imu_cb_accel(const float data[3], uint64_t timestamp_us) { send_data(acc, 3, 2); }
static void imu_cb_gyro(const float data[3], uint64_t timestamp_us) {
    send_data(gyro, 3, 3);
    T gyro_data = data[2] * (T)(PI / 180.0);
    kf.enqueMeasurement(MeasurementType::Gyro, &gyro_data, timestamp_us);
}
static void imu_cb_mag(const float data[3], uint64_t timestamp_us) { send_data(mag, 3, 4); }
static void imu_cb_gravity(const float data[3], uint64_t timestamp_us) { send_data(gravity, 3, 5); }
static void imu_cb_linear_accel(const float data[3], uint64_t timestamp_us) { send_data(acc_lin, 3, 6); }
static void imu_cb_rpy_orientation(const float data[3], uint64_t timestamp_us) { send_data(orpy, 3, 7); }
static void imu_cb_6DOF_orientation(const float data[4], uint64_t timestamp_us) { send_data(o6dof, 4, 8); }
static void imu_cb_9DOF_orientation(const float data[4], uint64_t timestamp_us) {
    send_data(o9dof, 4, 9);
    static SerialIO::TxUtil<float, 1, true> yaw_tx("yaw", 11);
    using std::atan2, std::asin;
    // T r = atan2(2 * (data[0] * data[1] + data[2] * data[3]), 1 - 2 * (data[1] * data[1] + data[2] * data[2]));
    // T p = asin(2 * (data[0] * data[2] - data[1] * data[3]));
    T y = atan2(2 * (data[0] * data[3] + data[1] * data[2]), 1 - 2 * (data[2] * data[2] + data[3] * data[3]));
    if (yaw_tx.txFinished()) {
        yaw_tx.setAll(y);
        wireless.send(yaw_tx);
    }
    kf.enqueMeasurement(MeasurementType::Yaw, &y, timestamp_us);
}
static void imu_cb_mag_orientation(const float data[4], uint64_t timestamp_us) { send_data(omag, 4, 10); }

void ICM20948::setupCallbacks() {
    // clang-format off
    if constexpr (use_uncal_mag       ) cb_uncal_mag        = imu_cb_uncal_mag       ;  // 未校准磁场
    if constexpr (use_uncal_gyro      ) cb_uncal_gyro       = imu_cb_uncal_gyro      ;  // 未校准陀螺仪
    if constexpr (use_accel           ) cb_accel            = imu_cb_accel           ;  // 加速度计
    if constexpr (use_gyro            ) cb_gyro             = imu_cb_gyro            ;  // 陀螺仪
    if constexpr (use_mag             ) cb_mag              = imu_cb_mag             ;  // 磁场
    if constexpr (use_gravity         ) cb_gravity          = imu_cb_gravity         ;  // 重力
    if constexpr (use_linear_accel    ) cb_linear_accel     = imu_cb_linear_accel    ;  // 线加速度
    if constexpr (use_rpy_orientation ) cb_rpy_orientation  = imu_cb_rpy_orientation ;  // 9DOF RPY
    if constexpr (use_6DOF_orientation) cb_6DOF_orientation = imu_cb_6DOF_orientation;  // 6DOF位姿
    if constexpr (use_9DOF_orientation) cb_9DOF_orientation = imu_cb_9DOF_orientation;  // 9DOF位姿
    if constexpr (use_mag_orientation ) cb_mag_orientation  = imu_cb_mag_orientation ;  // 地磁位姿
    // clang-format on
}