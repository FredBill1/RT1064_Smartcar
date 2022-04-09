#include "utils/FuncThread.hpp"
//
#include "devices.hpp"

static uint8_t id;

static float pre_speed = 0;

static float encoder_speed = 0;

static inline void SetMotorPwm() {
    static SerialIO::TxUtil<float, 2, true> encoderXfer("encoder", 0);
    float cur_speed;
    if (!wireless.getchar(id)) return;
    if (id == 4) {
        motorDrvL1.setPWM(0);
    } else {
        if (!wireless.getData<float>(cur_speed)) return;
        if (id == 0) {
            if (encoderXfer.txFinished()) {
                encoderXfer.setAll(pre_speed, encoder_speed);
                wireless.send(encoderXfer);
            }
            motorDrvL1.setPWM_Limit(cur_speed);
            pre_speed = cur_speed;
        }
    }
    beep.set(false);
}

static void updateWheelSpeed() {
    encoderL1.update();
    encoder_speed = encoderL1.get();
}

static void testMotorPwmEntry() {
    for (;;) {
        wireless.waitHeader();
        beep.set(true);
        if (!wireless.getchar(id)) continue;
        if (id == 1) SetMotorPwm();
        else
            beep.set(false);
    }
}

bool testMotorPwmNode() {
    return FuncTimer(updateWheelSpeed, "updateWheelSpeed", Param::MotorControlPeriod) &&
           FuncThread(testMotorPwmEntry, "testMotorPwm");
}