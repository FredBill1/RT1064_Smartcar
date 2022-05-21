#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_MT9V03X_CSI.h"
#include "common.h"
#include "fsl_debug_console.h"
}

#include "apriltag/apriltag.hpp"
#include "apriltag/undisort.hpp"
#include "apriltag/visualization.hpp"
#include "devices.hpp"

static void findRectEntry() {
    using namespace imgProc::apriltag;
    // AT_DTCM_SECTION_ALIGN(static uint8_t img[N * M], 64);

    int32_t pre_time = rt_tick_get();

    for (;;) {
        bool visualize = slave_switch[2].get();  // 拨码开关决定是否进行可视化，因为可视化会消耗时间

        // uint8_t* src = mt9v03x_csi_image_take();
        // undisort_I(src, img);  // 矫正图像畸变
        uint8_t* img = mt9v03x_csi_image_take();

        rects_t& rects = find_rects(img);
        if (visualize) {
            plot_rects(img, rects);
            show_plot_grayscale(img);
        }

        mt9v03x_csi_image_release();  // 释放图片

        int32_t cur_time = rt_tick_get();
        ips114_showint32(188, 0, cur_time - pre_time, 3);  // 显示耗时/ms
        pre_time = cur_time;
    }
}

bool findRectNode() { return FuncThread(findRectEntry, "findRect", 4096, 2, 1000); }
