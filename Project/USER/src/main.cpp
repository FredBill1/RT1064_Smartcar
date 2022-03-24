extern "C" {
#include "headfile.h"
int main(void);
}

#include "Thread.h"
#include "devices.hpp"
#include "nodes/nodes.hpp"

int main(void) {
    initDevices();
    // imu.init();
    // mt9v03x_csi_init();
    // usb_cdc_init();

    EnableGlobalIRQ(0);  // 启用全局中断
    wirelessNode();

    // imgUSBXferNode();
    // apriltagDetectNode();
    motorControlNode();

    // testMotorNode();
    // testQTimerNode();

    for (;;) {
        led.toggle();
        rt_thread_mdelay(500);
    }
}
