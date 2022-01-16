extern "C" {
#include "headfile.h"
int main(void);
}

#include <limits>

#include "Thread.h"
#include "devices.hpp"
#include "rosRT/Topic.hpp"
rt_timer_t fusionTimer;
void fusionTimerCB(void*) {}

void rotCB(const rosRT::msgs::QuaternionStamped& data) {
    wireless.writeV(data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w, data.header.stamp);
    wireless.sendTail();
}
auto rot = rosRT::Subscriber::create<rosRT::msgs::QuaternionStamped>("imu/6DOF_orientation", 1, rotCB);

namespace testtest {
void pose_kalman_test(void*);
}

int main(void) {
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    initDevices();
    EnableGlobalIRQ(0);
    // testtest::pose_kalman_test(NULL);

    fusionTimer = rt_timer_create("fusionTimer", fusionTimerCB, NULL, 20, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    rt_timer_start(fusionTimer);

    rtthread::Thread thread(testtest::pose_kalman_test, NULL, 20480, RT_THREAD_PRIORITY_MAX - 1, 20, "pose_kalman_test");
    thread.start();

    for (;;) {
        gpio_toggle(B9);
        rt_thread_mdelay(500);
        // fusionTimerCB(NULL);
    }
}
