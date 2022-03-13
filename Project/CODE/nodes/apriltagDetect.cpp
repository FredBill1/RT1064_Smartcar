#include "Thread.h"

extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_MT9V03X_CSI.h"
#include "common.h"
#include "fsl_debug_console.h"
#include "zf_gpio.h"
}

#include "apriltag/apriltag.hpp"
#include "apriltag/apriltag_pose.hpp"
#include "apriltag/tag25h9.hpp"
#include "apriltag/undisort.hpp"
#include "apriltag/visualization.hpp"
#include "apriltag/visualization_pose.hpp"
#include "devices.hpp"

// 相机和tag的参数在这里，识别用的参数在"apriltag/config.hpp"里
#include "ApriltagConfig.hpp"

static void apriltagDetectThreadEntry(void*) {
    using namespace imgProc::apriltag;
    AT_DTCM_SECTION_ALIGN(static uint8_t img[N * M], 64);

    // 初始化tag
    apriltag_family tf = tag25h9_create();
    tf.init(maxhamming);
    apriltag_detection_info info{nullptr, tagsize, fx, fy, cx, cy};  // 用来计算tag的位姿

    gpio_init(D4, GPI, 0, GPIO_PIN_CONFIG);
    int32_t pre_time = rt_tick_get();

    for (;;) {
        bool visualize = gpio_get(D4);  // 拨码开关决定是否进行可视化，因为可视化会消耗时间

        undisort_I(mt9v03x_csi_image_take(), img);  // 矫正图像畸变

        detections_t& dets = apriltag_detect(tf, img);  // 进行检测

        det_poses_t& det_poses = estimate_poses(info, dets);  // 计算tag位姿

        if (visualize) plot_det_poses(img, info, det_poses);  // 可视化

        if (visualize) show_plot_grayscale(img);  // 如果启用可视化就把图片显示出来

        mt9v03x_csi_image_release();  // 释放图片

        int32_t cur_time = rt_tick_get();
        ips114_showint32(188, 0, cur_time - pre_time, 3);  // 显示耗时/ms
        pre_time = cur_time;
    }
}

rtthread::Thread apriltagDetectThread(apriltagDetectThreadEntry, NULL, 4096, 1, 1000, "apriltagDetectThread");