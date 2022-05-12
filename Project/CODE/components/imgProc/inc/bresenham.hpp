#ifndef _imgProc_bresenham_hpp
#define _imgProc_bresenham_hpp

#include <algorithm>
#include <functional>

namespace imgProc {

template <typename drawPoint_t> inline void drawLine(int x1, int y1, int x2, int y2, drawPoint_t drawPoint) {
    int dx = std::abs(x2 - x1), sx = x1 < x2 ? 1 : -1, dy = -std::abs(y2 - y1), sy = y1 < y2 ? 1 : -1, err = dx + dy;
    for (;;) {
        drawPoint(x1, y1);
        if (x1 == x2 && y1 == y2) break;
        int e2 = 2 * err;
        if (e2 >= dy) err += dy, x1 += sx;
        if (e2 <= dx) err += dx, y1 += sy;
    }
}

template <typename drawPoint_t> inline void _drawCircleUtil(int xc, int yc, int x, int y, drawPoint_t &drawPoint) {
    drawPoint(xc + x, yc + y), drawPoint(xc - x, yc + y), drawPoint(xc + x, yc - y), drawPoint(xc - x, yc - y);
    drawPoint(xc + y, yc + x), drawPoint(xc - y, yc + x), drawPoint(xc + y, yc - x), drawPoint(xc - y, yc - x);
}

template <typename drawPoint_t> inline void drawCircle(int xc, int yc, int r, drawPoint_t drawPoint) {
    int x = 0, y = r;
    int d = 3 - (r << 1);
    _drawCircleUtil(xc, yc, x, y, drawPoint);
    while (y >= x) {
        ++x;
        if (d > 0) --y, d += ((x - y) << 2) + 10;
        else
            d += (x << 2) + 6;
        _drawCircleUtil(xc, yc, x, y, drawPoint);
    }
}

}  // namespace imgProc

#endif  // _imgProc_bresenham_hpp