#include "nodes.hpp"
//
#include "devices.hpp"

static void testMotorEntry() {
    int32_t duty = 5000;
    constexpr int32_t delta = 100;
    for (;;) {
        ips.printf("duty: %d\n", duty);
        rt_thread_mdelay(100);
        if (btn_c4.pressing()) duty += delta;
        else if (btn_c26.pressing())
            duty -= delta;
        else
            continue;
        motorDrvL1.setPWM(duty);
    }
}

bool testMotorNode() { return FuncThread(testMotorEntry, "testMotor"); }