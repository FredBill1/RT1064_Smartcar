extern "C" {
#include "headfile.h"
int main(void);
}

#include <limits>

#include "devices.hpp"
#include "rosRT/Topic.hpp"

rosRT::TopicT<int> topic_test(3);

static uint8 tail[4]{0x00, 0x00, 0x80, 0x7f};

#define PUTT(x) \
    for (int i = 0; i < sizeof(x); ++i) wireless.putchar(((uint8*)x)[i])
#define PUTV(x) \
    for (int i = 0; i < sizeof(x); ++i) wireless.putchar(((uint8*)&x)[i])
rt_timer_t fusionTimer;
void fusionTimerCB(void*) {
    // float tmp[4];
    // imu.ROTATION_VECTOR.get(tmp);
    // PUTT(tmp);
    // PUTT(tail);
}

int ttt = 0;
void pub(void*) {
    PRINTF("%d: pub  data: %d\r\n", rt_tick_get(), ++ttt);
    topic_test.publish(ttt);
}

void sub1(const int& data) { PRINTF("%d: sub1 data:%d\r\n", rt_tick_get(), data); }
auto sub2 = [](const int& data) -> void { PRINTF("%d: sub2 data:%d\r\n", rt_tick_get(), data); };
struct {
    void operator()(const int& data) { PRINTF("%d: sub3 data:%d\r\n", rt_tick_get(), data); }
} sub3;

int main(void) {
    //此处编写用户代码(例如：外设初始化代码等)

    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    // initDevices();
    topic_test.subscribe(sub1);
    topic_test.subscribe(sub2);
    topic_test.subscribe(sub3);

    EnableGlobalIRQ(0);

    fusionTimer =
        //     rt_timer_create("fusionTimer", fusionTimerCB, NULL, 20, RT_TIMER_FLAG_PERIODIC |
        //     RT_TIMER_FLAG_HARD_TIMER);
        // rt_timer_start(fusionTimer);
        rt_timer_create("fusionTimer", pub, NULL, 20, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    rt_timer_start(fusionTimer);
    for (;;) {
        gpio_toggle(B9);
        PRINTF("%d: main\r\n", rt_tick_get());
        rt_thread_mdelay(500);
        // fusionTimerCB(NULL);
    }
}
