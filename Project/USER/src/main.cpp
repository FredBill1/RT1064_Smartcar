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

int main(void) {
    //此处编写用户代码(例如：外设初始化代码等)

    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    icm20948.setMagnetometerBias(-143, -23, 180);
    icm20948.init();
    icm20948.selftest();
    // icm20948.enableSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD, 5);
    icm20948.enableSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR, 5);
    // icm20948.enableAllSensors(5);

    EnableGlobalIRQ(0);

    while (1) {
        //此处编写需要循环执行的代码
        icm20948.readSensor();
        // for (int i = 0; i < 4; ++i) { PRINTF("%.10f ", icm20948._quat9DOF[i]); }
        // PRINTF("\n\r");

        // PUTT(icm20948._mag);
        // PUTT(icm20948._quat9DOF);
        // PUTV(icm20948._quat9DOFaccuracy);

        // PUTT(icm20948._quat6DOF);
        // PUTV(icm20948._quat6DOFaccuracy);
        // PUTT(icm20948._mag);
        for (int i = 0; i < 4; ++i) PUTCHAR(tail[i]);
        // rt_thread_mdelay(5);

        // rt_thread_mdelay(1000);
    }
}
