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
    // testMotorPwmNode();
    // testKeyNode();
    // pose_kalman_test_node();
    // testPoseKalmanNode();
    // testLocalPlannerNode();

    poseKalmanNode();

    moveBase.set_enabled(true);
    rt_kprintf("start move base\r\n");
    rt_thread_mdelay(5000);
    baseDriver.setControlState(1, 1, 1, 1);
    rt_thread_mdelay(1000);
    moveBase.send_goal(10, 3, 3.14);

    // uartMasterTest();
    // uartSlaveTest();

    for (;;) {
        led.toggle();
        rt_thread_mdelay(500);
    }
}
