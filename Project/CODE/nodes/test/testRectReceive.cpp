#include "utils/FuncThread.hpp"
//
extern "C" {
#include "SEEKFREE_IPS114_SPI.h"
}
#include "RectConfig.hpp"
#include "bresenham.hpp"
#include "devices.hpp"

namespace imgProc {
namespace apriltag {

static Beep beep;

static uint8_t id;
static int rect_cnt;
static float_t rects[max_rect_cnt][2];
static int rect_cnt_copy;
static float_t rects_copy[max_rect_cnt][2];

constexpr float_t draw_xmin = 0.15;
constexpr float_t draw_xmax = 0.45;
constexpr float_t draw_ymin = -0.25;
constexpr float_t draw_ymax = 0.25;

static inline void draw_rect(const float_t r[2], uint16_t color) {
    int draw_y = N / 4 - (r[0] - draw_xmin) * (N / 4 / (draw_xmax - draw_xmin));
    int draw_x = M / 4 - (r[1] - draw_ymin) * (M / 4 / (draw_ymax - draw_ymin));
    auto plot = [color](int x, int y) {
        if (0 <= x && x < M / 4 && 0 <= y && y <= N / 4) ips114_drawpoint(x, y, color);
    };
    for (int r = 1; r <= 9; r += 2) drawCircle(draw_x, draw_y, r, plot);
}

static bool try_recv(SerialIO& uart) {
    uart.waitHeader();
    beep.set(true);
    if (!uart.getchar(id)) return false;
    if (!uart.getchar(id)) return false;
    rect_cnt = id / 2;
    if (rect_cnt > max_rect_cnt) return false;
    for (int i = 0; i < rect_cnt * 2; ++i)
        if (!uart.getData<float>(((float_t*)rects)[i])) return false;
    beep.set(false);
    return true;
}

static void testRectReceiveEntry() {
    static SerialIO::TxArr<float, max_rect_cnt * 2, true> rect_tx(33, "rect_tx");
    for (;;) {
        if (try_recv(uart3)) {
            for (int i = 0; i < rect_cnt_copy; ++i) draw_rect(rects_copy[i], 0xffff);
            rect_cnt_copy = rect_cnt;
            rt_memcpy(rects_copy, rects, sizeof(float_t[2]) * rect_cnt);

            if (rect_tx.txFinished()) {
                rect_tx.setArr(rects[0], rect_cnt * 2);
                wireless.send(rect_tx);
            }

            for (int i = 0; i < rect_cnt_copy; ++i) draw_rect(rects_copy[i], RED);
        }
    }
}
}  // namespace apriltag
}  // namespace imgProc
bool testRectReceiveNode() { return FuncThread(imgProc::apriltag::testRectReceiveEntry, "testRectReceive", 4096, 0); }