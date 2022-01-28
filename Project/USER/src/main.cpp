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
    wireless.writeV(data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w, data.header.stamp);
    wireless.sendTail();
}
auto rot = rosRT::Subscriber::create<rosRT::msgs::QuaternionStamped>("imu/6DOF_orientation", 1, rotCB);

int main(void) {
    gpio_init(B9, GPO, 0, GPIO_PIN_CONFIG);
    rt_thread_mdelay(500);

    ips114_init();  //初始化1.14寸IPS屏幕
    ips114_showstr(0, 0, "SEEKFREE MT9V03x");
    ips114_showstr(0, 1, "Initializing... ");

    //如果屏幕没有任何显示，请检查屏幕接线

    mt9v03x_csi_init();  //初始化摄像头 使用CSI接口
    //如果屏幕一直显示初始化信息，请检查摄像头接线
    //如果使用主板，一直卡在while(!uart_receive_flag)，请检查是否电池连接OK?
    //如果图像只采集一次，请检查场信号(VSY)是否连接OK?

    ips114_showstr(0, 1, "      OK...     ");
    systick_delay_ms(500);

    // wireless.init("Wireless", UART8_CONFIG);
    // initDevices();
    // wirelessThread.start();
    EnableGlobalIRQ(0);
    // testtest::pose_kalman_test(NULL);
    apriltagDetectThread.start();

    for (;;) {
        // wireless.waitHeader();
        // if (!wireless.readF(duty)) { ips.printf("Failed to read\n"); }
        // ips.printf("%d\n", duty);
        gpio_toggle(B9);
        rt_thread_mdelay(500);
    }
}
