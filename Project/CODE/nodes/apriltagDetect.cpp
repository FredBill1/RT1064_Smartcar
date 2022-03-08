#include "Thread.h"

extern "C" {
#include "SEEKFREE_MT9V03X_CSI.h"
#include "common.h"
#include "fsl_debug_console.h"
#include "zf_gpio.h"
}

//
#include "ApriltagConfig.h"
#include "apriltag/apriltag.hpp"
#include "apriltag/apriltag_pose.hpp"
#include "apriltag/tag25h9.hpp"
#include "apriltag/visualization.hpp"

static void apriltagDetectThreadEntry(void*) {
    using namespace imgProc::apriltag;
    auto tf = tag25h9_create();
    tf.init(1);
    apriltag_detection_info info{nullptr, tagsize, fx, fy, cx, cy};
    apriltag_pose solution;
    for (;;) {
        auto img = mt9v03x_csi_image_take();
        auto dets = apriltag_detect(tf, img);

        for (auto det_p : dets) {
            auto& det = *det_p;
            uint64_t color = 2333;
            plot_tag_det(img, det, color);
            info.det = det_p;
            estimate_pose_for_tag_homography(info, solution);
            PRINTF("x:%f y:%f z:%f\r\n", solution.t[0], solution.t[1], solution.t[2]);
        }
        show_plot_grayscale(img);
        mt9v03x_csi_image_release();
    }
}

rtthread::Thread apriltagDetectThread(apriltagDetectThreadEntry, NULL, 4096, 1, 1000, "apriltagDetectThread");