extern "C" {
#include "headfile.h"
int main(void);
}

#include <limits>

#include "Encoder.hpp"
#include "Icm20948.hpp"
#include "PinConfig.h"
#include "SerialIO.hpp"

ICM20948 icm20948(ICM20948_CONFIG);

static uint8 tail[4]{0x00, 0x00, 0x80, 0x7f};

#define PUTT(x) \
    for (int i = 0; i < sizeof(x); ++i) PUTCHAR(((uint8*)x)[i])
#define PUTV(x) \
    for (int i = 0; i < sizeof(x); ++i) PUTCHAR(((uint8*)&x)[i])
rt_timer_t fusionTimer;
extern "C" void fusionTimerCB(void*) {
    icm20948.readSensor();

    PUTT(icm20948._quat9DOF);
    PUTT(tail);
}

int main(void) {
    //此处编写用户代码(例如：外设初始化代码等)

    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    icm20948.setMagnetometerBias(-143, -23, 180);
    icm20948.init();
    icm20948.selftest();
    // icm20948.enableSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD, 5);
    icm20948.setSensorPeriod(INV_ICM20948_SENSOR_ROTATION_VECTOR, 20);
    icm20948.enableSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR);
    // icm20948.enableAllSensors(5);

    EnableGlobalIRQ(0);
    gpio_interrupt_init(C30, RISING, GPIO_INT_CONFIG);

    fusionTimer =
        rt_timer_create("fusionTimer", fusionTimerCB, NULL, 20, RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_HARD_TIMER);
    // rt_timer_start(fusionTimer);

    for (;;) {
        gpio_toggle(B9);
        rt_thread_mdelay(500);
    }
}
