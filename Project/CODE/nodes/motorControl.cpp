#include "utils/FuncThread.hpp"
//
#include "devices.hpp"

constexpr uint64_t cmd_vel_acc_timeout_us = 0.5 * 1e6;

static BaseDriver::ControlState state;

static inline void updateControlState() {
    if (baseDriver.loadControlState()) {
        const BaseDriver::ControlState &cur_state = baseDriver.getControlState();
        state ^= cur_state;
        if (state.L1()) motorCtrlL1.handleStateChange(cur_state.L1());
        if (state.L2()) motorCtrlL2.handleStateChange(cur_state.L2());
        if (state.R1()) motorCtrlR1.handleStateChange(cur_state.R1());
        if (state.R2()) motorCtrlR2.handleStateChange(cur_state.R2());
        state = cur_state;
    }
}

static inline void updateWheelSpeed() {
    baseDriver.loadCmdVelAcc();
    BaseDriver::CmdVelAcc &cmd_vel_acc = baseDriver.getCmdVelAcc();
    if (cmd_vel_acc.valid) {
        uint64_t dt_us = systick.get_us() - cmd_vel_acc.timestamp_us;
        if (dt_us > cmd_vel_acc_timeout_us) {
            cmd_vel_acc.valid = false;
            motorCtrlL1.setTargetSpeed(0);
            motorCtrlL2.setTargetSpeed(0);
            motorCtrlR1.setTargetSpeed(0);
            motorCtrlR2.setTargetSpeed(0);
            return;
        }
        float dt_s = dt_us * 1e-6f;
        float vX = cmd_vel_acc.vel.x + cmd_vel_acc.acc.x * dt_s;
        float vY = cmd_vel_acc.vel.y + cmd_vel_acc.acc.y * dt_s;
        float vYaw = cmd_vel_acc.vel.yaw + cmd_vel_acc.acc.yaw * dt_s;
        BaseDriver::WheelSpeed wheel = baseDriver.calc_vel(vX, vY, vYaw);
        motorCtrlL1.setTargetSpeed(wheel.L1);
        motorCtrlL2.setTargetSpeed(wheel.L2);
        motorCtrlR1.setTargetSpeed(wheel.R1);
        motorCtrlR2.setTargetSpeed(wheel.R2);
        return;
    }
    if (baseDriver.loadWheelSpeed()) {
        const BaseDriver::WheelSpeed &wheel = baseDriver.getWheelSpeed();
        motorCtrlL1.setTargetSpeed(wheel.L1);
        motorCtrlL2.setTargetSpeed(wheel.L2);
        motorCtrlR1.setTargetSpeed(wheel.R1);
        motorCtrlR2.setTargetSpeed(wheel.R2);
    }
}

static inline void readEncoder() {
    encoderL1.update();
    encoderL2.update();
    encoderR1.update();
    encoderR2.update();
    baseDriver.get_vel(encoderL1.get(), encoderL2.get(), encoderR1.get(), encoderR2.get());
}

static inline void updateKf() {
    using namespace pose_kalman;
    auto &baseSpeed = baseDriver.getBaseSpeed();
    T odom[3]{baseSpeed.x, baseSpeed.y, baseSpeed.yaw};
    kf.enqueMeasurement(MeasurementType::Odom, odom, systick.get_us());
}

static inline void applyMotorCtrl() {
    if (state.L1()) motorCtrlL1.update();
    if (state.L2()) motorCtrlL2.update();
    if (state.R1()) motorCtrlR1.update();
    if (state.R2()) motorCtrlR2.update();
}

static inline void uploadDebugData() {
    static SerialIO::TxUtil<float, 4, true> encoderXfer("encoder", 20);
    static SerialIO::TxUtil<float, 4, true> motorOutputXfer("motorOutput", 21);
    if (encoderXfer.txFinished()) {
        encoderXfer.setAll(encoderL1.get(), encoderL2.get(), encoderR1.get(), encoderR2.get());
        wireless.send(encoderXfer);
    }
    if (motorOutputXfer.txFinished()) {
        // motorOutputXfer.setAll(motorCtrlL1.getOutput(), motorCtrlL2.getOutput(), motorCtrlR1.getOutput(),
        //                        motorCtrlR2.getOutput());
        motorOutputXfer.setAll(motorCtrlL1.getTargetSpeed(), motorCtrlL2.getTargetSpeed(), motorCtrlR1.getTargetSpeed(),
                               motorCtrlR2.getTargetSpeed());
        wireless.send(motorOutputXfer);
    }
}

static void motorControlEntry() {
    updateControlState();
    updateWheelSpeed();
    readEncoder();
    updateKf();
    applyMotorCtrl();
    uploadDebugData();
}

bool motorControlNode() { return FuncTimer(motorControlEntry, "motorControl", Param::MotorControlPeriod); }