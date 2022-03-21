#include "Thread.h"
#include "devices.hpp"

static void testMotorThreadEntry(void*) {
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

rtthread::Thread testMotorThread(testMotorThreadEntry, NULL, 2048, (RT_THREAD_PRIORITY_MAX * 2) / 3, 1000, "testMotor");
