#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}

#include <algorithm>
#include <cmath>

#include "Camera.hpp"
#include "apriltag/RectSender.hpp"
#include "apriltag/apriltag.hpp"
#include "apriltag/reconcileRects.hpp"
#include "apriltag/undisort.hpp"
#include "apriltag/visualization.hpp"
#include "devices.hpp"
#include "edge_detect/A4Detect.hpp"
#include "edge_detect/show_edge.hpp"
//
#include "RectConfig.hpp"
#include "SlaveGlobalVars.hpp"

using namespace imgProc;
using namespace imgProc::apriltag;
using namespace imgProc::edge_detect;

#define SWITCH_SKIP                         \
    {                                       \
        if (slave_switch[2].get()) {        \
            ips114_showstr(188, 6, "SKIP"); \
            show_grayscale(img);            \
            return;                         \
        } else                              \
            ips114_showstr(188, 6, "    "); \
    }

static int32_t pre_time;
static uint8_t* img;
static int A4_pre_cnt = -1, A4_cnt_same = 0;

static inline void Reset() {
    ips114_clear(WHITE);
    slaveGlobalVars.set_state(SlaveGlobalVars::A4_PREPARE);
}

static inline void A4Prepare() {
    SWITCH_SKIP;
    A4Detect(img, 7, 5, 50, 100);
    show_edge(img);
    if (slave_key[0].pressing()) {
        slaveGlobalVars.set_state(SlaveGlobalVars::A4);
        A4_pre_cnt = -1;
    }
}

static inline void A4Detect() {
    SWITCH_SKIP;
    static SerialIO::TxArr<float, target_coords_maxn * 2, true> a4_tx(32, "a4_tx");

    bool res = A4Detect(img, 7, 5, 50, 100);
    if (!res) A4_pre_cnt = -1;

    if (target_coords_cnt == A4_pre_cnt) {
        if (++A4_cnt_same >= 3) {
            A4_cnt_same = 3;
            a4_tx.txFinished(-1);
            a4_tx.setArr(target_coords_corr[0], target_coords_cnt * 2);
            uart3.send(a4_tx);
        }
    } else {
        A4_pre_cnt = target_coords_cnt, A4_cnt_same = 1;
    }

    show_edge(img);  // 显示边缘图片
}

static inline void FindRect() {
    SWITCH_SKIP;
    static RectSender rectSender(33);

    // bool show_thresh = slave_key[0].get();  // 是否显示二值化图
    bool show_thresh = 0;
    bool visualize = !show_thresh && slave_switch[1].get();  // 是否显示效果

    rects_t& rects = find_rects(img, min_magnitude,
                                show_thresh ? apriltag_detect_visualize_flag::threshim : apriltag_detect_visualize_flag::None);
    if (visualize) plot_rects(img, rects, GREEN);

    reconcileRects(rects);

    if (!rects.empty()) rectSender.send_to(rects, uart3);

    if (visualize) {
        plot_rects(img, rects, RED);
        show_plot_grayscale(img);
    }
}

static inline void Idle() { show_grayscale(img); }

static void slaveMainLoopEntry() {
    int32_t pre_time = rt_tick_get();

    for (;;) {
        img = camera.snapshot();

        auto state = slaveGlobalVars.get_state();
        ips114_showstr(188, 7, slaveGlobalVars.state_str(state));

        switch (state) {
        case SlaveGlobalVars::IDLE: Idle(); break;
        case SlaveGlobalVars::RESET: Reset(); break;
        case SlaveGlobalVars::A4_PREPARE: A4Prepare(); break;
        case SlaveGlobalVars::A4: A4Detect(); break;
        case SlaveGlobalVars::RECT: FindRect(); break;
        default: show_grayscale(img); break;
        }

        camera.release();
        int32_t cur_time = rt_tick_get();
        ips114_showint32(188, 0, cur_time - pre_time, 3);  // 显示耗时/ms
        pre_time = cur_time;
    }
}

bool slaveMainLoopNode() { return FuncThread(slaveMainLoopEntry, "slaveMainLoop", 4096, 2, 1000); }
