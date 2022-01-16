#include <rthw.h>
#include <rtthread.h>
#define _USE_MATH_DEFINES
#include <cmath>

#include "crt.h"
extern "C" {
#include "fsl_debug_console.h"
}

#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/UnscentedKalmanFilter.hpp"
#include "pose_kalman/inc/IMUMeasurementModel.hpp"
#include "pose_kalman/inc/SystemModel.hpp"

namespace testtest {
using T = double;
using State = Planar_Robot::State<T>;
using Control = Planar_Robot::Control<T>;
using SystemModel = Planar_Robot::SystemModel<T>;

using IMUMeasurement = Planar_Robot::IMUMeasurement<T>;
using IMUModel = Planar_Robot::IMUMeasurementModel<T>;

void pose_kalman_test(void*) {
    State x;
    Control u;
    SystemModel sys;

    Kalman::ExtendedKalmanFilter<State> predictor;
    Kalman::ExtendedKalmanFilter<State> ekf;
    Kalman::UnscentedKalmanFilter<State> ukf;

    x.setZero();
    x.ox() = 1;

    IMUModel imuM;

    predictor.init(x);
    ekf.init(x);
    ukf.init(x);
    for (;;) {
        for (int i = 0; i < 100; i++) {
            u.vX() = 1;
            u.vY() = 1;
            u.vYaw() = 1;
            u.dt() = 0.02;

            x = sys.f(x, u);

            auto x_pred = predictor.predict(sys, u);
            // auto x_ekf = ekf.predict(sys, u);
            auto x_ukf = ukf.predict(sys, u);

            {
                IMUMeasurement imu = imuM.h(x);
                // x_ekf = ekf.update(imuM, imu);
                x_ukf = ukf.update(imuM, imu);
            }
            PRINTF("pre %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n\r", x_pred.x(), x_pred.y(), x_pred.ox(), x_pred.oy(),
                   x_pred.vX(), x_pred.vY(), x_pred.vYaw(), x_pred.aX(), x_pred.aY());
            // PRINTF("ekf %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n\r", x_ekf.x(), x_ekf.y(), x_ekf.ox(), x_ekf.oy(),
            //        x_ekf.vX(), x_ekf.vY(), x_ekf.vYaw(), x_ekf.aX(), x_ekf.aY());
            PRINTF("ukf %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n\r", x_ukf.x(), x_ukf.y(), x_ukf.ox(), x_ukf.oy(),
                   x_ukf.vX(), x_ukf.vY(), x_ukf.vYaw(), x_ukf.aX(), x_ukf.aY());

            PRINTF("%d\n\r", i);
        }
        rt_thread_mdelay(100);
    }
}
}  // namespace testtest