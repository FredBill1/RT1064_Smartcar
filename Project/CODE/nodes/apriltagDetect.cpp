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
    // 这两个用来计算tag的位姿
    apriltag_detection_info info{nullptr, tagsize, fx, fy, cx, cy};
    apriltag_pose solution;

    int32_t pre_time = rt_tick_get();
    gpio_init(D4, GPI, 0, GPIO_PIN_CONFIG);

    for (;;) {
        uint8_t* src = mt9v03x_csi_image_take();  // 获取图片
        undisort_I(src, img);                     // 矫正图像畸变

        bool visualize = gpio_get(D4);  // 拨码开关决定是否进行可视化，因为可视化会消耗时间

        // 进行检测
        detections_t& dets = apriltag_detect(tf, img);

        // 检测结果是用std::forward_list保存的
        // 识别tag用的内存是统一管理的，所以这里处理完也不需要释放这些指针
        for (apriltag_detection* det_p : dets) {  // 遍历结果
            apriltag_detection& det = *det_p;
            info.det = det_p;

            // 计算tag的位姿，并将结果存入solution
            estimate_pose_for_tag_homography(info, solution);

            if (visualize) {                          // 如果进行可视化，就在原图上进行修改
                plot_tag_det(img, det);               // 画出tag边框和id
                plot_pose_axis(img, info, solution);  // 画出tag的坐标系，RGB分别为xyz轴
                // plot_pose_cube(img, info, solution); // 画出tag的3d方块
            }

            // TODO 进行处理
        }
        if (visualize) show_plot_grayscale(img);  // 如果启用可视化就把图片显示出来
        mt9v03x_csi_image_release();              // 释放图片
        int32_t cur_time = rt_tick_get();
        ips114_showint32(188, 0, cur_time - pre_time, 3);  // 显示耗时/ms
        pre_time = cur_time;
    }
}

rtthread::Thread apriltagDetectThread(apriltagDetectThreadEntry, NULL, 4096, 1, 1000, "apriltagDetectThread");