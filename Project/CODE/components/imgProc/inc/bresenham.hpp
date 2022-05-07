#ifndef _imgProc_bresenham_hpp
#define _imgProc_bresenham_hpp

#include <functional>

namespace imgProc {

using drawPoint_t = std::function<void(int, int)>;

void drawLine(int x1, int y1, int x2, int y2, drawPoint_t drawPoint);

void drawCircle(int x, int y, int r, drawPoint_t drawPoint);

}  // namespace imgProc

#endif  // _imgProc_bresenham_hpp