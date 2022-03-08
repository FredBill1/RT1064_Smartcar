extern "C" {
#include "headfile.h"
int main(void);
}

#include "Thread.h"
#include "devices.hpp"
#include "nodes/nodes.hpp"

int main(void) {
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(C4, GPI, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    // wireless.init("Wireless", UART8_CONFIG);
    initDevices();
    wirelessThread.start();
    // imu.init();
    mt9v03x_csi_init();
    // usb_cdc_init();
    EnableGlobalIRQ(0);

    // imgUSBXferThread.start();

    // testtest::pose_kalman_test(NULL);
    apriltagDetectThread.start();
    for (;;) {
        gpio_toggle(B9);
        rt_thread_mdelay(500);
    }
}
