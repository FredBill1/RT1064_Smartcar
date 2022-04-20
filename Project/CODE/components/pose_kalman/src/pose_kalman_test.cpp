#include <rtthread.h>

#include <random>

#include "devices.hpp"
#include "utils/FuncThread.hpp"
//

#include "kalman/SquareRootExtendedKalmanFilter.hpp"
#include "pose_kalman/PosMeasurementModel.hpp"
#include "pose_kalman/SystemModel.hpp"
#include "pose_kalman/VYawMeasurementModel.hpp"

namespace pose_kalman {
static T noise(T factor) {
    static std::default_random_engine generator;
    static std::normal_distribution<T> distribution(0, 1);
    return factor * distribution(generator);
}

constexpr T xy_Noise = 0.2;
constexpr T yaw_Noise = 0.2;

constexpr T vYaw_m_Noise = 0.2;

void pose_kalman_test() {
    using imgProc::apriltag::atan2f;
    static SerialIO::TxUtil<float, 6, true> x_tx("x", 30);
    static SerialIO::TxUtil<float, 6, true> x_kf_tx("x_kf", 31);
    State x, x_kf;
    Control u, u_real;
    SystemModel sys;
    VYawMeasurementModel vYaw_m;
    PosMeasurementModel pos_m;

    Kalman::SquareRootExtendedKalmanFilter<State> kf;

    x.setZero();
    x.ox() = 1;
    x.vX() = 1;
    x.vYaw() = 1;

    kf.init(x);
    u.vX() = u_real.vX() = 1;
    u.vY() = u_real.vY() = 0;
    u.vYaw() = u_real.vYaw() = 1;
    u.dt() = u_real.dt() = 0.005;
    for (;;) {
        u.x() = u_real.vX() + noise(xy_Noise);
        u.y() = u_real.vY() + noise(xy_Noise);
        u.vYaw() = u_real.vYaw() + noise(yaw_Noise);

        x = sys.f(x, u_real);
        x_kf = kf.predict(sys, u);

        VYawMeasurement vYaw_h = vYaw_m.h(x);
        vYaw_h.vYaw() += noise(vYaw_m_Noise);
        x_kf = kf.update(vYaw_m, vYaw_h);

        // PosMeasurement pos_h = pos_m.h(x);
        // x_kf = kf.update(pos_m, pos_h);

        if (x_tx.txFinished()) {
            x_tx.setAll(x.x(), x.y(), atan2f(x.oy(), x.ox()), x.vX(), x.vY(), x.vYaw());
            wireless.send(x_tx);
        }
        if (x_kf_tx.txFinished()) {
            x_kf_tx.setAll(x_kf.x(), x_kf.y(), atan2f(x_kf.oy(), x_kf.ox()), x_kf.vX(), x_kf.vY(), x_kf.vYaw());
            wireless.send(x_kf_tx);
        }
        rt_thread_mdelay(5);
    }
}
}  // namespace pose_kalman

bool pose_kalman_test_node() { return FuncThread(pose_kalman::pose_kalman_test, "pose_kalman_test", 6144); }
