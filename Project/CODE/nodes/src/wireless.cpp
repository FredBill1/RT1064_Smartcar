#include "utils/FuncThread.hpp"
//
#include <cmath>

#include "MasterGlobalVars.hpp"
#include "devices.hpp"
#include "masterConfig.hpp"
#include "pose_kalman/params.hpp"

static uint8_t id;
static Beep beep;

static inline void SystemReset() {
    beep.set(false);
    NVIC_SystemReset();
}

static inline void SetMotorPwm() {
    bool use_deadzone = id == 2;
    int32 speed;
    if (!wireless.getchar(id)) return;
    if (id == 4) {
        beep.set(false);

        motorDrvL1.setPWM(0, false);
        motorDrvL2.setPWM(0, false);
        motorDrvR1.setPWM(0, false);
        motorDrvR2.setPWM(0, false);
        return;
    }
    if (!wireless.getData<float>(speed)) return;
    beep.set(false);

    switch (id) {
    case 0: motorDrvL1.setPWM_Limit(speed, use_deadzone); break;
    case 1: motorDrvL2.setPWM_Limit(speed, use_deadzone); break;
    case 2: motorDrvR1.setPWM_Limit(speed, use_deadzone); break;
    case 3: motorDrvR2.setPWM_Limit(speed, use_deadzone); break;
    }
}

static inline void SetMotorControlState() {
    uint8_t state;
    if (!wireless.getchar(state)) return;
    beep.set(false);

    baseDriver.setControlState(state);
}

static inline void SetMotorControllerParam() {
    bool use_wc = id == 4;  // 决定是使用控制器带宽wc还是分开指定kp和kd
    if (!wireless.getchar(id)) return;
    if (use_wc) {
        float wc, wo, b0;
        if (!wireless.getData<float>(wc, wo, b0)) return;
        beep.set(false);

        // 先禁用控制器, 确保安全
        baseDriver.setControlState(0);
        rt_thread_mdelay(Param::MotorControlPeriod * 10);

        switch (id) {
        case 0: controllerL1.setParameters(wc, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        case 1: controllerL2.setParameters(wc, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        case 2: controllerR1.setParameters(wc, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        case 3: controllerR2.setParameters(wc, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        }
    } else {
        // float kp, kd, wo, b0;
        // if (!wireless.getData<float>(kp, kd, wo, b0)) return;
        // beep.set(false);

        // // 先禁用控制器, 确保安全
        // baseDriver.setControlState(0);
        // rt_thread_mdelay(Param::MotorControlPeriod * 10);

        // switch (id) {
        // case 0: controllerL1.setParameters(kp, kd, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        // case 1: controllerL2.setParameters(kp, kd, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        // case 2: controllerR1.setParameters(kp, kd, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        // case 3: controllerR2.setParameters(kp, kd, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        // }
    }
}

static inline void SetMotorTargetSpeed() {
    if (!wireless.getchar(id)) return;
    if (id == 4) {
        beep.set(false);
        baseDriver.cmd_vel(0, 0, 0, 0);
        return;
    }
    float speed;
    if (!wireless.getData<float>(speed)) return;
    beep.set(false);

    switch (id) {
    case 0:
        baseDriver.cmd_vel(speed, 0, 0, 0);
        baseDriver.setControlState(1, 0, 0, 0);
        break;
    case 1:
        baseDriver.cmd_vel(0, speed, 0, 0);
        baseDriver.setControlState(0, 1, 0, 0);
        break;
    case 2:
        baseDriver.cmd_vel(0, 0, speed, 0);
        baseDriver.setControlState(0, 0, 1, 0);
        break;
    case 3:
        baseDriver.cmd_vel(0, 0, 0, speed);
        baseDriver.setControlState(0, 0, 0, 1);
        break;
    }
}

static inline void Remote() {
    uint8_t forward, left, rotate;
    if (!(wireless.getchar(forward) && wireless.getchar(left) && wireless.getchar(rotate))) return;
    beep.set(false);
    constexpr float x_speed = 1;
    constexpr float y_speed = 1;
    constexpr float r_speed = 2;
    float x = forward ? (forward == 1 ? x_speed : -x_speed) : 0;
    float y = left ? (left == 1 ? y_speed : -y_speed) : 0;
    float r = rotate ? (rotate == 1 ? r_speed : -r_speed) : 0;
    baseDriver.cmd_vel(x, y, r);
}

static inline void SendGoal() {
    float x, y, yaw;
    if (!(wireless.getData<float>(x, y, yaw))) return;
    beep.set(false);
    auto goal = MoveBase::Goal::getDefault();
    goal.x = x, goal.y = y, goal.yaw = yaw;
    moveBase.send_goal(goal);
}

static inline void SendState() {
    MoveBase::State state;
    if (!wireless.getArr<float, 6>(state.state)) return;
    beep.set(false);
    state.timestamp_us = systick.get_us();
    moveBase.send_set_state(state);
}

static inline void SetLocalPlannerParam() {
    using namespace pose_kalman;
    LocalPlanner::Params params;
    if (!(wireless.getData<float>(params.vel_lim_xy, params.vel_lim_yaw, params.acc_lim_xy, params.acc_lim_yaw, params.dt_ref)))
        return;
    beep.set(false);
    localPlanner.setParams(params);
}

static inline void SetServo() {
    if (!wireless.getchar(id)) return;
    auto& srv = id == 0 ? srv_l : srv_r;
    float degree;
    if (!wireless.getData<float>(degree) || degree < 0 || degree > srv.max_angle) return;
    beep.set(false);
    srv.set(degree);
}

static inline void ConfigCamera() {
    if (!wireless.getchar(id)) return;
    float data;
    if (!wireless.getData<float>(data)) return;
    beep.set(false);
    static SerialIO::TxUtil<float, 1, true> cam_tx("cam_tx", 0);
    cam_tx.set_id(id);
    cam_tx.setAll(data);
    cam_tx.txFinished(-1);
    slave_uart.send(cam_tx);
}

void recvCoords(SerialIO& uart, Beep& beep);

static inline void SendMasterStop() {
    beep.set(false);
    masterGlobalVars.signal_reset();
    moveBase.set_enabled(false);
    baseDriver.cmd_vel(0, 0, 0, 0);
    rt_thread_mdelay(pose_kalman::predict_period_us * 10 / 1000);
    moveBase.set_enabled(true);
}

static void wirelessEntry() {
    for (;;) {
        wireless.waitHeader();
        beep.set(true);
        if (!wireless.getchar(id)) continue;
        switch (id) {
        case 0: SystemReset(); break;
        case 1:
        case 2: SetMotorPwm(); break;
        case 3: SetMotorControlState(); break;
        case 4:
        case 5: SetMotorControllerParam(); break;
        case 6: SetMotorTargetSpeed(); break;
        case 7: Remote(); break;
        case 8: SendGoal(); break;
        case 9: SetLocalPlannerParam(); break;
        case 10: SendState(); break;
        case 11: SetServo(); break;
        case 12: ConfigCamera(); break;

        case 32: recvCoords(wireless, beep); break;
        case 253: SendMasterStop(); break;
        }
    }
}

bool wirelessNode() { return FuncThread(wirelessEntry, "wireless", 4096, Thread::uart_priority); }