extern "C" {
#include "headfile.h"
int main(void);
}

#include "devices.hpp"
#include "nodes/nodes.hpp"

int main(void) {
    initDevices();
    // imu.init();
    mt9v03x_csi_init();
    usb_cdc_init();

    EnableGlobalIRQ(0);  // ����ȫ���ж�
    wirelessNode();

    // imgUSBXferNode();
    // apriltagDetectNode();
    // findRectNode();
    // motorControlNode();

    // testMotorNode();
    // testQTimerNode();
    // testMotorPwmNode();
    // testKeyNode();
    // pose_kalman_test_node();
    // testPoseKalmanNode();
    // testLocalPlannerNode();
    // testTSPSolverNode();
    testCannyNode();

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
