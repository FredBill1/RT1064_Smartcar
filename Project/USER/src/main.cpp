extern "C" {
#include "headfile.h"
int main(void);
}

#include <cmath>
#include <limits>

#include "Thread.h"
#include "apriltag/visualization.hpp"
#include "devices.hpp"
#include "nodes/nodes.hpp"
#include "rosRT/Topic.hpp"

void imu6DOFCB(const rosRT::msgs::QuaternionStamped& data) {
    static SerialIO::TxUtil<float, 4, true> txUtil("imu6DOF", 0);
    if (txUtil.txFinished()) {
        txUtil.setAll(data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w);
        wireless.send(txUtil);
    }
}
auto imu6DOFSub = rosRT::Subscriber::create<rosRT::msgs::QuaternionStamped>("imu/6DOF_orientation", 1, imu6DOFCB);

void imuLinAccCB(const rosRT::msgs::Vector3Stamped& data) {
    static SerialIO::TxUtil<float, 3, true> txUtil("imuLinAcc", 1);
    if (txUtil.txFinished()) {
        txUtil.setAll(data.vector.x, data.vector.y, data.vector.z);
        wireless.send(txUtil);
    }
}
auto imuLinAccSub = rosRT::Subscriber::create<rosRT::msgs::Vector3Stamped>("imu/linear_accel", 1, imuLinAccCB);

void camEntry(void*) {
    using namespace imgProc::apriltag;
    static SerialIO::TxUtil<uint8_t, N * M, false, false> txUtil("cam");
    for (;;) {
        uint8_t* img = mt9v03x_csi_image_take();
        if (txUtil.txFinished()) rt_memcpy(txUtil.Data(), img, N * M), wireless.send(txUtil), rt_kprintf("frame\r\n");
        show_grayscale(img);
        mt9v03x_csi_image_release();
    }
}

int main(void) {
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    // wireless.init("Wireless", UART8_CONFIG);
    initDevices();
    wirelessThread.start();
    // imu.init();
    mt9v03x_csi_init();
    EnableGlobalIRQ(0);

    rtthread::Thread camthread(camEntry, NULL, 2048, 3, 100, "cam");
    camthread.start();

    // testtest::pose_kalman_test(NULL);
    // apriltagDetectThread.start();
    for (;;) {
        gpio_toggle(B9);
        rt_thread_mdelay(500);
    }
}
