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

void printImu(const rosRT::msgs::QuaternionStamped& data) {
    auto time = rt_tick_get_millisecond();
    wireless.writeV(data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w, data.header.stamp,
                    time);
    wireless.sendTail();
}
auto sub = rosRT::Subscriber::create<rosRT::msgs::QuaternionStamped>("imu/9DOF_orientation", 1, printImu);
int main(void) {
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    initDevices();
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
