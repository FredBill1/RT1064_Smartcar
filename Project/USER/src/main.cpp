extern "C" {
#include "headfile.h"
int main(void);
}

#include <limits>

#include "devices.hpp"

static uint8 tail[4]{0x00, 0x00, 0x80, 0x7f};

#define PUTT(x) \
    for (int i = 0; i < sizeof(x); ++i) wireless.putchar(((uint8*)x)[i])
#define PUTV(x) \
    for (int i = 0; i < sizeof(x); ++i) wireless.putchar(((uint8*)&x)[i])
rt_timer_t fusionTimer;
void fusionTimerCB(void*) {
    float tmp[4];
    imu.ROTATION_VECTOR.get(tmp);
    PUTT(tmp);
    PUTT(tail);
}

int main(void) {
    //此处编写用户代码(例如：外设初始化代码等)

    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    initDevices();

    EnableGlobalIRQ(0);

    // fusionTimer =
    //     rt_timer_create("fusionTimer", fusionTimerCB, NULL, 20, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
    // rt_timer_start(fusionTimer);
    for (;;) {
        // gpio_toggle(B9);
        // rt_thread_mdelay(500);
        fusionTimerCB(NULL);
    }
}
