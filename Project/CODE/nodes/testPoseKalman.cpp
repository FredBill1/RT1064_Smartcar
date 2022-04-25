#include "utils/FuncThread.hpp"
//
#include "pose_kalman/NoiseGenerator.hpp"
#include "pose_kalman/PoseKalman.hpp"
//
#include "apriltag/fmath.hpp"
#include "devices.hpp"
namespace pose_kalman {
static constexpr T sys_xy_sigma2 = 0.05;
static constexpr T sys_yaw_sigma2 = 0.06;
static constexpr T sys_v_xy_sigma2 = 0.025;
static constexpr T sys_v_yaw_sigma2 = 0.02;

static constexpr T odom_v_xy_sigma2 = 1;
static constexpr T odom_v_yaw_sigma2 = 3;

static constexpr T gyro_v_yaw_sigma2 = 1;
static void testPoseKalmanEntry() {
    static SerialIO::TxUtil<float, 6, true> x_tx("x", 30);
    static SerialIO::TxUtil<float, 6, true> x_kf_tx("x_kf", 31);
    static SerialIO::TxUtil<float, 6, true> x_kf_vYaw_tx("x_kf_gyro", 32);

    static PoseKalman real, odom_only, full;

    static NoiseGenerator odom_v_xy_noise(odom_v_xy_sigma2);
    static NoiseGenerator odom_v_yaw_noise(odom_v_yaw_sigma2);
    static NoiseGenerator gyro_v_yaw_noise(gyro_v_yaw_sigma2);

    {
        T sysCov[6][6]{0};
        sysCov[0][0] = sys_xy_sigma2;
        sysCov[1][1] = sys_xy_sigma2;
        sysCov[2][2] = sys_yaw_sigma2;
        sysCov[3][3] = sys_v_xy_sigma2;
        sysCov[4][4] = sys_v_xy_sigma2;
        sysCov[5][5] = sys_v_yaw_sigma2;
        real.setSystemCovariance(sysCov[0]);
        odom_only.setSystemCovariance(sysCov[0]);
        full.setSystemCovariance(sysCov[0]);
    }
    {
        T odomCov[3][3]{0};
        odomCov[0][0] = odomCov[1][1] = odomCov[2][2] = 1e-6;
        real.setMeasurementCovariance(MeasurementType::Odom, odomCov[0]);
        odomCov[0][0] = odom_v_xy_sigma2;
        odomCov[1][1] = odom_v_xy_sigma2;
        odomCov[2][2] = odom_v_yaw_sigma2;
        odom_only.setMeasurementCovariance(MeasurementType::Odom, odomCov[0]);
        full.setMeasurementCovariance(MeasurementType::Odom, odomCov[0]);
    }
    {
        T gyroCov[1][1]{0};
        gyroCov[0][0] = gyro_v_yaw_sigma2;
        real.setMeasurementCovariance(MeasurementType::Gyro, gyroCov[0]);
        odom_only.setMeasurementCovariance(MeasurementType::Gyro, gyroCov[0]);
        full.setMeasurementCovariance(MeasurementType::Gyro, gyroCov[0]);
    }
    {
        T state[6]{0};
        real.setState(state);
        odom_only.setState(state);
        full.setState(state);
    }
    {
        real.setEnabled(true);
        odom_only.setEnabled(true);
        full.setEnabled(true);
    }
    constexpr uint64_t dt_us = 1000;
    for (uint64_t t = 0;; t += dt_us) {
        using imgProc::apriltag::sinf, imgProc::apriltag::cosf;
        T vX = 2 * (cosf(t * 2e-6) + 1);
        T vY = 3 * sinf(t * 3e-6);
        T vYaw = 3 * sinf(t * 3e-7);
        {
            T odom_m[3]{vX, vY, vYaw};
            real.enqueMeasurement(MeasurementType::Odom, odom_m, t);
            odom_m[0] += odom_v_xy_noise();
            odom_m[1] += odom_v_xy_noise();
            odom_m[2] += odom_v_yaw_noise();
            odom_only.enqueMeasurement(MeasurementType::Odom, odom_m, t);
            full.enqueMeasurement(MeasurementType::Odom, odom_m, t);
        }
        {
            T gyro_m[1]{vYaw + gyro_v_yaw_noise()};
            full.enqueMeasurement(MeasurementType::Gyro, gyro_m, t);
        }
        {
            real.update(t);
            odom_only.update(t);
            full.update(t);
        }

        if (x_tx.txFinished()) {
            x_tx.setArr(real.getState());
            wireless.send(x_tx);
        }
        if (x_kf_tx.txFinished()) {
            x_kf_tx.setArr(odom_only.getState());
            wireless.send(x_kf_tx);
        }
        if (x_kf_vYaw_tx.txFinished()) {
            x_kf_vYaw_tx.setArr(full.getState());
            wireless.send(x_kf_vYaw_tx);
        }
        rt_thread_mdelay(1);
    }
}

}  // namespace pose_kalman

bool testPoseKalmanNode() { return FuncThread(pose_kalman::testPoseKalmanEntry, "testPoseKalman", 15360); }