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
        motorDrvL1.setPWM(0, false);
        motorDrvL2.setPWM(0, false);
        motorDrvR1.setPWM(0, false);
        motorDrvR2.setPWM(0, false);
    } else {
        if (!wireless.getData<float>(speed)) return;
        switch (id) {
        case 0: motorDrvL1.setPWM_Limit(speed, use_deadzone); break;
        case 1: motorDrvL2.setPWM_Limit(speed, use_deadzone); break;
        case 2: motorDrvR1.setPWM_Limit(speed, use_deadzone); break;
        case 3: motorDrvR2.setPWM_Limit(speed, use_deadzone); break;
        }
    }
    beep.set(false);
}

static inline void SetMotorControlState() {
    uint8_t state;
    if (!wireless.getchar(state)) return;
    moveBase.setControlState(state);
    beep.set(false);
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
        }
    }
}

bool wirelessNode() { return FuncThread(wirelessEntry, "wireless", 1024, 0); }