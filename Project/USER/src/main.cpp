extern "C" {
#include "headfile.h"
int main(void);
}

#include <limits>

#include "Thread.h"
#include "devices.hpp"
#include "nodes/nodes.hpp"
#include "rosRT/Topic.hpp"

//
#include "apriltag/apriltag.hpp"
#include "apriltag/internal/UnionBuffer.hpp"
#include "apriltag/internal/fit_quad.hpp"
#include "apriltag/internal/segmentation.hpp"
#include "apriltag/internal/threshold.hpp"
#include "apriltag/tag25h9.hpp"
AT_SDRAM_SECTION_ALIGN(imgProc::apriltag::QuadImg_t binary, 64);

void rotCB(const rosRT::msgs::QuaternionStamped& data) {
    wireless.writeV(data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w, data.header.stamp);
    wireless.sendTail();
}
auto rot = rosRT::Subscriber::create<rosRT::msgs::QuaternionStamped>("imu/6DOF_orientation", 1, rotCB);

void imgThreadEntry(void*) {
    using namespace imgProc;
    using namespace imgProc::apriltag;
    for (;;) {
        auto p = mt9v03x_csi_image_take();
        // ips114_displayimage032(p[0], MT9V03X_CSI_W, MT9V03X_CSI_H);  //显示摄像头图像
        imgProc::apriltag::threshold(p[0], binary);

        unionfind_connected(binary);

        // show_unionfind();

        auto clusters = gradient_clusters(binary);

        // ips << binary;
        // show_clusters(*clusters);

        auto tf = tag25h9_create();
        auto quads = fit_quads(*clusters, tf, p[0]);

        // ips114_displayimage032(p[0], MT9V03X_CSI_W, MT9V03X_CSI_H);  //显示摄像头图像
        for (auto& quad : *quads) {
            for (int i = 0; i < 4; ++i) binary.plot(quad.p[i][1], quad.p[i][0]);
        }
        ips << binary;

        rt_kprintf("%d\r\n", std::distance(quads->begin(), quads->end()));

        // if (staticBuffer.overflow()) rt_kprintf("overflowed\r\n");
        // else
        //     rt_kprintf("used: %dB %dKB %dMB\r\n", staticBuffer.usage(), staticBuffer.usage() >> 10, staticBuffer.usage() >>
        //     20);
        mt9v03x_csi_image_release(p);
        // rt_thread_mdelay(100);
    }
}

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

    // initDevices();
    // wirelessThread.start();
    EnableGlobalIRQ(0);
    // testtest::pose_kalman_test(NULL);

    rtthread::Thread imgThread(imgThreadEntry, NULL, 4096, RT_THREAD_PRIORITY_MAX - 1, 20, "imgThread");
    imgThread.start();

    for (;;) {
        // wireless.waitHeader();
        // if (!wireless.readF(duty)) { ips.printf("Failed to read\n"); }
        // ips.printf("%d\n", duty);
        gpio_toggle(B9);
        rt_thread_mdelay(500);
    }
}
