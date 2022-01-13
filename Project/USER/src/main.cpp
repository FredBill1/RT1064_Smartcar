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

int ttt = 0;
rosRT::Publisher pub1("tes2", sizeof(int), 3);
rosRT::Publisher* pub2;
void pub(void*) {
    PRINTF("%d: pub1  data: %d\r\n", rt_tick_get(), ttt);
    pub1.publish(&ttt);
    int lll = ttt + 100;
    PRINTF("%d: pub2  data: %d\r\n", rt_tick_get(), lll);
    pub2->publish(&lll);
    ++ttt;
    // PRINTF("%d: pub  data: %d\r\n", rt_tick_get(), ++ttt);
}

void sub1(const void* data) { PRINTF("%d: sub1 data:%d\r\n", rt_tick_get(), *(int*)data); }
auto sub2 = [](const void* data) -> void { PRINTF("%d: sub2 data:%d\r\n", rt_tick_get(), *(int*)data); };
struct {
    void operator()(const void* data) { PRINTF("%d: sub3 data:%d\r\n", rt_tick_get(), *(int*)data); }
} sub3;
rosRT::Subscriber test_sub3("test", sizeof(int), 3, sub3);

int main(void) {
    //此处编写用户代码(例如：外设初始化代码等)

    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);
    wireless.init("Wireless", UART8_CONFIG);
    // initDevices();
    EnableGlobalIRQ(0);
    pub2 = new rosRT::Publisher("test", sizeof(int), 3);
    rosRT::Subscriber test_sub1("test", sizeof(int), 3, sub1);
    rosRT::Subscriber test_sub2("tes2", sizeof(int), 3, sub2);

    fusionTimer =
        //     rt_timer_create("fusionTimer", fusionTimerCB, NULL, 20, RT_TIMER_FLAG_PERIODIC |
        //     RT_TIMER_FLAG_HARD_TIMER);
        // rt_timer_start(fusionTimer);
        rt_timer_create("fusionTimer", pub, NULL, 200, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    rt_timer_start(fusionTimer);
    for (;;) {
        gpio_toggle(B9);
        PRINTF("%d: main\r\n", rt_tick_get());
        rt_thread_mdelay(500);
        wireless.writeV(10, 20, 30);
        wireless.sendTail();
        // fusionTimerCB(NULL);
    }
}
