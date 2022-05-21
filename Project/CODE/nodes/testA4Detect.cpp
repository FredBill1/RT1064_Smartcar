#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_MT9V03X_CSI.h"
}

#include "apriltag/visualization.hpp"
#include "devices.hpp"
#include "edge_detect/A4Detect.hpp"
#include "edge_detect/show_edge.hpp"

namespace imgProc {
namespace edge_detect {
using namespace apriltag;

static void testA4DetectEntry() {
    int32_t pre_time = rt_tick_get();

    for (;;) {
        bool enabled = slave_switch[2].get();  // ≤¶¬Îø™πÿ

        uint8_t* img = mt9v03x_csi_image_take();

        if (enabled) {
            A4Detect(img, 50, 100);
            show_edge(img);  // œ‘ æ±ﬂ‘µÕº∆¨
        } else {
            show_grayscale(img);  // œ‘ æª“∂»Õº
        }

        mt9v03x_csi_image_release();  //  Õ∑≈Õº∆¨

        int32_t cur_time = rt_tick_get();
        ips114_showint32(188, 0, cur_time - pre_time, 3);  // œ‘ æ∫ƒ ±/ms
        pre_time = cur_time;
    }
}
}  // namespace edge_detect
}  // namespace imgProc

bool testA4DetectNode() { return FuncThread(imgProc::edge_detect::testA4DetectEntry, "testA4Detect", 4096, 2, 1000); }
