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
#include "fieldParam.hpp"
#include "slaveConfig.hpp"

using namespace imgProc;
using namespace imgProc::apriltag;
using namespace imgProc::edge_detect;

static int32_t pre_time;
static uint8_t* img;
static SlaveGlobalVars::State state;
static int canny_thresh[2]{20, 30};

namespace keys {
static bool skip;
static bool visualize;
static bool binary;
};  // namespace keys
#define SKIP_CHECK           \
    if (keys::skip) {        \
        show_grayscale(img); \
        return;              \
    }

static inline void keyScan() {
    using namespace keys;
    skip = slave_switch[2].get();
    visualize = slave_switch[1].get();
    binary = slave_switch[0].get();

    if (slave_key[0].pressing()) canny_thresh[0] += 5;
    if (slave_key[1].pressing()) canny_thresh[0] -= 5;
    if (slave_key[2].pressing()) canny_thresh[1] += 5;
    if (slave_key[3].pressing()) canny_thresh[1] -= 5;

    ips114_showstr(188, 6, skip ? "SKIP" : "    ");

    if (state == SlaveGlobalVars::A4) {
        ips114_showuint16(188, 4, (uint16)canny_thresh[0]);
        ips114_showuint16(188, 5, (uint16)canny_thresh[1]);
    } else {
        ips114_showstr(188, 5, visualize ? "VIS" : "   ");
        ips114_showstr(188, 4, binary ? "BIN" : "   ");
    }
}

static inline void Reset() {
    ips114_clear(WHITE);
    slaveGlobalVars.set_state(SlaveGlobalVars::A4);
}

static inline void A4Detect() {
    SKIP_CHECK;
    static SerialIO::TxArr<float, target_coords_maxn * 2, true> a4_tx(32, "a4_tx");

    bool res = A4Detect(img, borderWidth, borderHeight, canny_thresh[0], canny_thresh[1]);
    if (res) {
        if constexpr (squareAlign)
            for (int i = 0; i < target_coords_cnt; ++i)
                for (int j = 0; j < 2; ++j)
                    target_coords_corr[i][j] = (int(target_coords_corr[i][j] / squareSize) + 0.5f) * squareSize;
        a4_tx.txFinished(-1);
        a4_tx.setArr(target_coords_corr[0], target_coords_cnt * 2);
        master_uart.send(a4_tx);
    }

    show_edge(img);  // ��ʾ��ԵͼƬ
}

static inline void FindRect() {
    SKIP_CHECK;
    static RectSender rectSender(33);

    bool visualize = !keys::binary && keys::visualize;  // �Ƿ���ʾЧ��

    rects_t& rects = find_rects(img, min_magnitude,
                                keys::binary ? apriltag_detect_visualize_flag::threshim : apriltag_detect_visualize_flag::None);
    if (visualize) plot_rects(img, rects, GREEN);

    reconcileRects(rects);

    rectSender.send_to(rects, master_uart);

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

        state = slaveGlobalVars.get_state();
        ips114_showstr(188, 7, slaveGlobalVars.state_str(state));
        keyScan();

        switch (state) {
        case SlaveGlobalVars::IDLE: Idle(); break;
        case SlaveGlobalVars::RESET: Reset(); break;
        case SlaveGlobalVars::A4: A4Detect(); break;
        case SlaveGlobalVars::RECT: FindRect(); break;
        default: show_grayscale(img); break;
        }

        camera.release();
        int32_t cur_time = rt_tick_get();
        ips114_showint32(188, 0, cur_time - pre_time, 3);  // ��ʾ��ʱ/ms
        pre_time = cur_time;
    }
}

bool slaveMainLoopNode() { return FuncThread(slaveMainLoopEntry, "slaveMainLoop", 4096, Thread::highest_priority + 2, 1000); }
