#include <rtthread.h>

#include <random>

#include "devices.hpp"
#include "utils/FuncThread.hpp"
//

#include "apriltag/fmath.hpp"
#include "kalman/SquareRootExtendedKalmanFilter.hpp"
#include "pose_kalman/SystemModel.hpp"
#include "pose_kalman/measurementTypes.hpp"

namespace pose_kalman {
using imgProc::apriltag::atan2f, imgProc::apriltag::sinf, imgProc::apriltag::cosf;

class NoiseGenerator {
    static std::default_random_engine generator;
    std::normal_distribution<T> distribution;

 public:
    NoiseGenerator(T sigma2 = 1) : distribution(0, sigma2) {}
    T operator()() { return distribution(generator); }
};
std::default_random_engine NoiseGenerator::generator;

constexpr T xy_Noise = 0.05;
constexpr T yaw_Noise = 0.06;
constexpr T v_xy_Noise = 0.025;
constexpr T v_yaw_Noise = 0.02;

constexpr T v_xy_m_Noise = 1;
constexpr T v_yaw_m_Noise = 3;

constexpr T gyro_m_Noise = 1;

// NoiseGenerator xy_noise_gen(xy_Noise);
// NoiseGenerator yaw_noise_gen(yaw_Noise);
// NoiseGenerator v_xy_noise_gen(v_xy_Noise);
// NoiseGenerator v_yaw_noise_gen(v_yaw_Noise);

NoiseGenerator v_xy_m_noise_gen(v_xy_m_Noise);
NoiseGenerator v_yaw_m_noise_gen(v_yaw_m_Noise);

NoiseGenerator gyro_m_noise_gen(gyro_m_Noise);

void pose_kalman_test() {
    using imgProc::apriltag::atan2f;
    static SerialIO::TxUtil<float, 6, true> x_tx("x", 30);
    static SerialIO::TxUtil<float, 6, true> x_kf_tx("x_kf", 31);
    static SerialIO::TxUtil<float, 6, true> x_kf_vYaw_tx("x_kf_gyro", 32);
    State x, x_kf, x_kf_gyro;
    Control u;
    SystemModel sys;
    Gyro::Model gryo_m;
    Odom::Model v_m;

    Kalman::SquareRootExtendedKalmanFilter<State> kf, kf_gyro;

    if (0) {  // System Model Covariance
        auto sysCovariance = sys.getCovariance();
        using S = State;
        sysCovariance(S::X, S::X) = xy_Noise;
        sysCovariance(S::Y, S::Y) = xy_Noise;
        sysCovariance(S::YAW, S::YAW) = yaw_Noise;
        sysCovariance(S::V_X, S::V_X) = v_xy_Noise;
        sysCovariance(S::V_Y, S::V_Y) = v_xy_Noise;
        sysCovariance(S::V_YAW, S::V_YAW) = v_yaw_Noise;
        sys.setCovariance(sysCovariance);
    }

    {  // Odom Measurement Model Covariance
        auto vMCovariance = v_m.getCovariance();
        using M = Odom::Data;
        vMCovariance(M::V_X, M::V_X) = v_xy_m_Noise;
        vMCovariance(M::V_Y, M::V_Y) = v_xy_m_Noise;
        vMCovariance(M::V_YAW, M::V_YAW) = v_yaw_m_Noise;
        v_m.setCovariance(vMCovariance);
    }

    {  // Gyro Measurement Model Covariance
        auto vYawCovariance = gryo_m.getCovariance();
        using M = Gyro::Data;
        vYawCovariance(M::V_YAW, M::V_YAW) = gyro_m_Noise;
        gryo_m.setCovariance(vYawCovariance);
    }

    // init state
    x.setZero();
    kf.init(x);
    kf_gyro.init(x);

    u.dt() = 0.001;
    for (T t = 0;; t += u.dt()) {
        using imgProc::apriltag::sinf, imgProc::apriltag::cosf;
        // simulate true system state
        x.vX() = 3 * sinf(t * 0.2);
        x.vY() = 3 * sinf(t * 0.3);
        x.vYaw() = 3 * sinf(t * 0.5);
        x = sys.f(x, u);

        // simulate measurement
        Odom::Data v_h = v_m.h(x);
        Gyro::Data gyro_h = gryo_m.h(x);

        // add noise to measurement
        v_h.vX() += v_xy_m_noise_gen();
        v_h.vY() += v_xy_m_noise_gen();
        v_h.vYaw() += v_yaw_m_noise_gen();
        gyro_h.vYaw() += gyro_m_noise_gen();

        // odom only
        kf.update(v_m, v_h);
        x_kf = kf.predict(sys, u);

        // with gyro
        kf_gyro.update(v_m, v_h);
        kf_gyro.update(gryo_m, gyro_h);
        x_kf_gyro = kf_gyro.predict(sys, u);

        if (x_tx.txFinished()) {
            x_tx.setAll(x.x(), x.y(), x.yaw(), x.vX(), x.vY(), x.vYaw());
            wireless.send(x_tx);
        }
        if (x_kf_tx.txFinished()) {
            x_kf_tx.setAll(x_kf.x(), x_kf.y(), x_kf.yaw(), x_kf.vX(), x_kf.vY(), x_kf.vYaw());
            wireless.send(x_kf_tx);
        }
        if (x_kf_vYaw_tx.txFinished()) {
            x_kf_vYaw_tx.setAll(x_kf_gyro.x(), x_kf_gyro.y(), x_kf_gyro.yaw(), x_kf_gyro.vX(), x_kf_gyro.vY(), x_kf_gyro.vYaw());
            wireless.send(x_kf_vYaw_tx);
        }
        rt_thread_mdelay(1);
    }
}
}  // namespace pose_kalman

bool pose_kalman_test_node() { return FuncThread(pose_kalman::pose_kalman_test, "pose_kalman_test", 10240); }
