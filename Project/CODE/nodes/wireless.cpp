#include "nodes.hpp"
//
#include "devices.hpp"

static uint8_t id;

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

    moveBase.setControlState(state);
}

static inline void SetMotorControllerParam() {
    bool use_wc = id == 4;  // 决定是使用控制器带宽wc还是分开指定kp和kd
    if (!wireless.getchar(id)) return;
    if (use_wc) {
        float wc, wo, b0;
        if (!wireless.getData<float>(wc, wo, b0)) return;
        beep.set(false);

        // 先禁用控制器, 确保安全
        moveBase.setControlState(0);
        rt_thread_mdelay(Param::MotorControlPeriod * 10);

        switch (id) {
        case 0: controllerL1.setParameters(wc, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        case 1: controllerL2.setParameters(wc, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        case 2: controllerR1.setParameters(wc, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        case 3: controllerR2.setParameters(wc, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        }
    } else {
        float kp, kd, wo, b0;
        if (!wireless.getData<float>(kp, kd, wo, b0)) return;
        beep.set(false);

        // 先禁用控制器, 确保安全
        moveBase.setControlState(0);
        rt_thread_mdelay(Param::MotorControlPeriod * 10);

        switch (id) {
        case 0: controllerL1.setParameters(kp, kd, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        case 1: controllerL2.setParameters(kp, kd, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        case 2: controllerR1.setParameters(kp, kd, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        case 3: controllerR2.setParameters(kp, kd, wo, b0, Param::MotorControlPeriod * 0.001f); break;
        }
    }
}

static inline void SetMotorTargetSpeed() {
    if (!wireless.getchar(id)) return;
    if (id == 4) {
        beep.set(false);
        moveBase.cmd_vel(0, 0, 0, 0);
        return;
    }
    float speed;
    if (!wireless.getData<float>(speed)) return;
    beep.set(false);

    MoveBase::WheelSpeed wheelSpeed;
    wheelSpeed.setZero();
    switch (id) {
    case 0: wheelSpeed.L1 = speed; break;
    case 1: wheelSpeed.L2 = speed; break;
    case 2: wheelSpeed.R1 = speed; break;
    case 3: wheelSpeed.R2 = speed; break;
    }
    moveBase.cmd_vel(wheelSpeed);
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
        }
    }
}

bool wirelessNode() { return FuncThread(wirelessEntry, "wireless", 4096, 0); }