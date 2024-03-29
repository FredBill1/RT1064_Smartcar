extern "C" {
#include "headfile.h"
int main(void);
}

#include "MCU_ID.hpp"
#include "MasterGlobalVars.hpp"
#include "devices.hpp"
#include "nodes/nodes.hpp"
#include "show_image.hpp"

int main(void) {
    initDevices();
    MCU_MASTER masterGlobalVars.use_9dof = master_switch[0].get();
    MCU_MASTER if (!master_switch[1].get()) imu.init();
    MCU_SLAVE camera.init();
    MCU_SLAVE usb_cdc_init();

    EnableGlobalIRQ(0);  // ����ȫ���ж�
    // show_image::koishi();

    // MCU_SLAVE imgUSBXferNode();
    MCU_SLAVE uartSlaveNode();
    MCU_SLAVE slaveMainLoopNode();

    MCU_MASTER wirelessNode();
    MCU_MASTER motorControlNode();
    MCU_MASTER uartMasterNode();
    MCU_MASTER uartArtNode();
    MCU_MASTER masterMainLoopNode();
    MCU_MASTER poseKalmanNode();
    // MCU_MASTER sendTraverseNode();

    MCU_MASTER moveBase.set_enabled(true);
    MCU_MASTER baseDriver.setControlState(1, 1, 1, 1);

    // MCU_BOTH testKeyNode();

    // MCU_SLAVE testA4DetectNode();
    // MCU_SLAVE testFindRectNode();
    // MCU_SLAVE testCannyNode();
    // MCU_SLAVE uartSlaveTest();
    // MCU_SLAVE apriltagDetectNode();

    // MCU_MASTER testArmDrvNode();
    // MCU_MASTER testMagnetNode();
    // MCU_MASTER testRectGrabNode();
    // MCU_MASTER testRectReceiveNode();
    // MCU_MASTER testMotorNode();
    // MCU_MASTER testQTimerNode();
    // MCU_MASTER testMotorPwmNode();
    // MCU_MASTER testPoseKalmanNode();
    // MCU_MASTER testLocalPlannerNode();
    // MCU_MASTER testTSPSolverNode();
    // MCU_MASTER testA4ReceiveNode();
    // MCU_MASTER uartMasterTest();
    // MCU_MASTER testArtNode();

    for (;;) {
        led.toggle();
        rt_thread_mdelay(500);
    }
}
