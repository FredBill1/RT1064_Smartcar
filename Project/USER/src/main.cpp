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

int main(void) {
    //此处编写用户代码(例如：外设初始化代码等)

    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    icm20948.init();
    icm20948.enableAllSensors(5);

    EnableGlobalIRQ(0);

    while (1) {
        //此处编写需要循环执行的代码
        icm20948.readSensor();
        for (int i = 0; i < 4; ++i) { PRINTF("%.10f ", icm20948._quat9DOF[i]); }
        PRINTF("\n\r");
        // rt_thread_mdelay(100);

        // rt_thread_mdelay(1000);
    }
}
