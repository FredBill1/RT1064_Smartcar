#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}

#include "Camera.hpp"
#include "apriltag/visualization.hpp"
#include "devices.hpp"
#include "edge_detect/A4Detect.hpp"
#include "edge_detect/show_edge.hpp"
//
#include "fieldParam.hpp"

namespace imgProc {
namespace edge_detect {
using namespace apriltag;

static void testA4DetectEntry() {
    static SerialIO::TxArr<float, target_coords_maxn * 2, true> a4_tx(32, "a4_tx");

    int32_t pre_time = rt_tick_get();

    for (;;) {
        bool enabled = slave_switch[2].get();  // ≤¶¬Îø™πÿ

        uint8_t* img = camera.snapshot();

        if (enabled) {
            bool res = A4Detect(img, borderWidth, borderHeight, 50, 100);
            if (res) {
                a4_tx.txFinished(-1);
                a4_tx.setArr(target_coords_corr[0], target_coords_cnt * 2);
                uart3.send(a4_tx);
            }
            show_edge(img);  // œ‘ æ±ﬂ‘µÕº∆¨
        } else {
            show_grayscale(img);  // œ‘ æª“∂»Õº
        }

        camera.release();  //  Õ∑≈Õº∆¨

        int32_t cur_time = rt_tick_get();
        ips114_showint32(188, 0, cur_time - pre_time, 3);  // œ‘ æ∫ƒ ±/ms
        pre_time = cur_time;
    }
}
}  // namespace edge_detect
}  // namespace imgProc

bool testA4DetectNode() { return FuncThread(imgProc::edge_detect::testA4DetectEntry, "testA4Detect", 4096, 2, 1000); }
