#include "Thread.h"

extern "C" {
#include "SEEKFREE_MT9V03X_CSI.h"
#include "common.h"
#include "fsl_debug_console.h"
#include "zf_gpio.h"
}

//
#include "apriltag/apriltag.hpp"
#include "apriltag/apriltag_pose.hpp"
#include "apriltag/tag25h9.hpp"
#include "apriltag/visualization.hpp"

static void apriltagDetectThreadEntry(void*) {
    using namespace imgProc::apriltag;
    AT_SDRAM_NONCACHE_SECTION_ALIGN(static uint8_t img[N * M], 64);
    auto tf = tag25h9_create();
    tf.init(1);
    auto pre = rt_tick_get_millisecond();
    apriltag_detection_info info{nullptr, 10, 652.3019409179688, 653.458251953125, 312.8074714771901, 230.0874313802706};
    apriltag_pose solution;
    gpio_init(D4, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(D27, GPI, 0, GPIO_PIN_CONFIG);
    gpio_init(C4, GPI, 0, GPIO_PIN_CONFIG);
    for (;;) {
        mt9v03x_csi_image_take(img);
        auto dets = apriltag_detect(tf, img);

        for (auto det_p : dets) {
            auto& det = *det_p;
            // PRINTF("id: %d\r\nhanmming: %d, decision_margin: %f\r\n", det.id, det.hamming, det.decision_margin);
            uint64_t color = 2333;
            // PRINTF("center: x=%f y=%f\r\n", det.c[0], det.c[1]);
            plot(det.c[1], det.c[0], color & 0xFFFF);
            for (int i = 0; i < 4; i++) {
                color *= int(1e9 + 7);
                // PRINTF("p%d: x=%f y=%f\r\n", i, det.p[i][0], det.p[i][1]);
                plot(det.p[i][1], det.p[i][0], color & 0xFFFF);
            }
            info.det = det_p;
            estimate_pose_for_tag_homography(info, solution);
            auto& R = solution.R;
            // float xr = std::atan2(R[2][1], R[2][2]);
            // float yr = std::atan2(-R[2][0], std::sqrt(R[2][1] * R[2][1] + R[2][2] * R[2][2]));
            // float zr = std::atan2(R[1][0], R[0][0]);
            // wireless.writeV(xr, yr, zr);
            // wireless.sendTail();
            PRINTF("x:%f y:%f z:%f\r\n", solution.t[0], solution.t[1], solution.t[2]);
        }

        PRINTF("\r\n");
        while (gpio_get(D4) && gpio_get(C4)) {}
    }
}

rtthread::Thread apriltagDetectThread(apriltagDetectThreadEntry, NULL, 4096, 1, 1000, "apriltagDetectThread");