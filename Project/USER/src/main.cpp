extern "C" {
#include "headfile.h"
int main(void);
}

#include <limits>

#include "devices.hpp"
#include "rosRT/Topic.hpp"

rt_timer_t fusionTimer;
void fusionTimerCB(void*) {
    // float tmp[4];
    // imu.ROTATION_VECTOR.get(tmp);
    // PUTT(tmp);
    // PUTT(tail);
}

void printImu(const void* data) {
    const rosRT::msgs::QuaternionStamped& dat = *(rosRT::msgs::QuaternionStamped*)data;
    auto time = systick_getval_us() / 1000;
    wireless.writeV(dat.quaternion.x, dat.quaternion.y, dat.quaternion.z, dat.quaternion.w, dat.header.stamp, time);
    wireless.sendTail();
}

int main(void) {
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    initDevices();
    rosRT::Subscriber sub("imu/9DOF_orientation", sizeof(rosRT::msgs::QuaternionStamped), 1, printImu);
    EnableGlobalIRQ(0);

    fusionTimer =
        rt_timer_create("fusionTimer", fusionTimerCB, NULL, 20, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    rt_timer_start(fusionTimer);

    for (;;) {
        gpio_toggle(B9);
        rt_thread_mdelay(500);
        // fusionTimerCB(NULL);
    }
}
