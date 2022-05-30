extern "C" {
#include "headfile.h"
int main(void);
}

#include "devices.hpp"
#include "nodes/nodes.hpp"
#include "show_image.hpp"

int main(void) {
    initDevices();
    // imu.init();
    // camera.init();
    usb_cdc_init();

    EnableGlobalIRQ(0);  // 启用全局中断
    // show_image::koishi();
    wirelessNode();

    // imgUSBXferNode();
    // apriltagDetectNode();
    // motorControlNode();
    uartMasterNode();
    mainLoopNode();

    // testFindRectNode();
    // testRectReceiveNode();
    // testMotorNode();
    // testQTimerNode();
    // testMotorPwmNode();
    // testKeyNode();
    // pose_kalman_test_node();
    // testPoseKalmanNode();
    // testLocalPlannerNode();
    // testTSPSolverNode();
    // testCannyNode();
    // testA4DetectNode();
    // testA4ReceiveNode();

    // poseKalmanNode();

    // moveBase.set_enabled(true);
    // baseDriver.setControlState(1, 1, 1, 1);

    // uartMasterTest();
    // uartSlaveTest();

    for (;;) {
        led.toggle();
        rt_thread_mdelay(500);
    }
}
