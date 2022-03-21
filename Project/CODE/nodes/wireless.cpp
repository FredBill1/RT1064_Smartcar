#include "nodes.hpp"
//
#include "devices.hpp"

static uint8_t id;

inline void SystemReset() {
    beep.set(false);
    NVIC_SystemReset();
}

inline void SetMotorPwm() {
    int32_t speed;
    if (!(wireless.getchar(id) && wireless.getData<float>(speed))) return;
    switch (id) {
    case 0: motorDrvL1.setPWM(speed); break;
    case 1: motorDrvL2.setPWM(speed); break;
    case 2: motorDrvR1.setPWM(speed); break;
    case 3: motorDrvR2.setPWM(speed); break;
    }
    beep.set(false);
}

static void wirelessEntry() {
    for (;;) {
        wireless.waitHeader();
        beep.set(true);
        if (!wireless.getchar(id)) continue;
        switch (id) {
        case 0: SystemReset(); break;
        case 1: SetMotorPwm(); break;
        }
    }
}

bool wirelessNode() { return FuncThread(wirelessEntry, "wireless", 1024, 0); }