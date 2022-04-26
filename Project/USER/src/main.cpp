extern "C" {
#include "headfile.h"
int main(void);
}

#include "Thread.h"
#include "devices.hpp"
#include "nodes/nodes.hpp"
bool pose_kalman_test_node();
int main(void) {
    initDevices();
    imu.init();
    // mt9v03x_csi_init();
    // usb_cdc_init();

    EnableGlobalIRQ(0);  // 启用全局中断
    wirelessNode();

    // imgUSBXferNode();
    // apriltagDetectNode();
    motorControlNode();

    // testMotorNode();
    // testQTimerNode();
    // testMotorPwmNode();
    // testKeyNode();
    // pose_kalman_test_node();
    // testPoseKalmanNode();

    poseKalmanNode();

    // uartMasterTest();
    // uartSlaveTest();

    for (;;) {
        led.toggle();
        rt_thread_mdelay(500);
    }
}
