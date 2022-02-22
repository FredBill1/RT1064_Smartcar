extern "C" {
#include "headfile.h"
int main(void);
}

#include <cmath>
#include <limits>

#include "Thread.h"
#include "devices.hpp"
#include "nodes/nodes.hpp"
#include "rosRT/Topic.hpp"

void rotCB(const rosRT::msgs::QuaternionStamped& data) {
    static SerialIO::TxUtil<float, 4> txUtil("imu6DOF");
    if (txUtil.txFinished()) {
        txUtil.setAll(data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w);
        wireless.send(txUtil);
    }
}
auto rot = rosRT::Subscriber::create<rosRT::msgs::QuaternionStamped>("imu/6DOF_orientation", 1, rotCB);

SerialIO::TxUtil<float, 3> data("txtest");

int main(void) {
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    // wireless.init("Wireless", UART8_CONFIG);
    initDevices();
    wirelessThread.start();
    EnableGlobalIRQ(0);
    // testtest::pose_kalman_test(NULL);
    // apriltagDetectThread.start();
    for (;;) {
        gpio_toggle(B9);
        rt_thread_mdelay(500);
    }
}
