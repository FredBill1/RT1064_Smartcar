#include "utils/FuncThread.hpp"
//

#include "devices.hpp"
#include "pose_kalman/params.hpp"

namespace pose_kalman {

static inline void setupSystemCovariance() {
    T sysCov[6][6]{0};
    sysCov[0][0] = sys_xy_sigma2;
    sysCov[1][1] = sys_xy_sigma2;
    sysCov[2][2] = sys_yaw_sigma2;
    sysCov[3][3] = sys_v_xy_sigma2;
    sysCov[4][4] = sys_v_xy_sigma2;
    sysCov[5][5] = sys_v_yaw_sigma2;
    kf.setSystemCovariance(sysCov[0]);
}

static inline void setupOdomCovariance() {
    T odomCov[3][3]{0};
    odomCov[0][0] = odom_v_xy_sigma2;
    odomCov[1][1] = odom_v_xy_sigma2;
    odomCov[2][2] = odom_v_yaw_sigma2;
    kf.setMeasurementCovariance(MeasurementType::Odom, odomCov[0]);
}

static inline void setupGyroCovariance() {
    T gyroCov[1][1]{0};
    gyroCov[0][0] = gyro_v_yaw_sigma2;
    kf.setMeasurementCovariance(MeasurementType::Gyro, gyroCov[0]);
}

static inline void setupPredictCovariance() {
    T predictCov[6][6]{0};
    predictCov[0][0] = 0.1;
    predictCov[1][1] = 0.1;
    predictCov[2][2] = 0.1;
    predictCov[3][3] = 0.1;
    predictCov[4][4] = 0.1;
    predictCov[5][5] = 0.1;
    kf.setPredictionCovariance(predictCov[0]);
}

static inline void setInitialState() {
    T state[6]{0};
    kf.setState(state);
}

static void poseKalmanEntry() {
    static SerialIO::TxUtil<float, 6, true> pose_tx("pose", 30);
    setupSystemCovariance();
    setupOdomCovariance();
    setupGyroCovariance();
    // setupPredictCovariance();
    setInitialState();
    rt_thread_mdelay(100);
    kf.setEnabled(true);
    for (;;) {
        kf.update(systick.get_us());
        const T* state = kf.getState();

        if (pose_tx.txFinished()) {
            pose_tx.setArr(state);
            wireless.send(pose_tx);
        }

        int64_t delta_time = predict_period_us - systick.get_us();
        delta_time /= 1000;
        if (delta_time > 0) rt_thread_mdelay(delta_time);
    }
}

}  // namespace pose_kalman

bool poseKalmanNode() { return FuncThread(pose_kalman::poseKalmanEntry, "poseKalman", 15360); }