#include "Thread.h"

extern "C" {
#include "SEEKFREE_MT9V03X_CSI.h"
#include "common.h"
#include "fsl_debug_console.h"
#include "zf_gpio.h"
}

//
#include "ApriltagConfig.hpp"
#include "apriltag/apriltag.hpp"
#include "apriltag/apriltag_pose.hpp"
#include "apriltag/tag25h9.hpp"
#include "apriltag/visualization.hpp"
#include "apriltag/visualization_pose.hpp"

static void apriltagDetectThreadEntry(void*) {
    using namespace imgProc::apriltag;
    apriltag_family tf = tag25h9_create();
    tf.init(1);
    apriltag_detection_info info{nullptr, tagsize, fx, fy, cx, cy};
    apriltag_pose solution;
    int32_t pre_time = rt_tick_get();
    for (;;) {
        uint8_t* img = mt9v03x_csi_image_take();
        detections_t& dets = apriltag_detect(tf, img);

        for (apriltag_detection* det_p : dets) {
            apriltag_detection& det = *det_p;
            plot_tag_det(img, det);
            info.det = det_p;
            estimate_pose_for_tag_homography(info, solution);
            plot_pose_axis(img, info, solution);
        }
        show_plot_grayscale(img);
        mt9v03x_csi_image_release();
        int32_t cur_time = rt_tick_get();
        rt_kprintf("%d\r\n", cur_time - pre_time);
        pre_time = cur_time;
    }
}

rtthread::Thread apriltagDetectThread(apriltagDetectThreadEntry, NULL, 4096, 1, 1000, "apriltagDetectThread");