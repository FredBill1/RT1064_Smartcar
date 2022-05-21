#ifndef _imgProc_CoordStack_hpp
#define _imgProc_CoordStack_hpp

#include <cstdint>

#include "imgProc/common.hpp"

namespace imgProc {

class CoordStack {
    int cnt = 0;

 public:
    void push(int16_t x, int16_t y) { push({x, y}); }
    void push(Coordinate x);
    Coordinate pop();
    bool empty();
};

}  // namespace imgProc

#endif  // _imgProc_CoordStack_hpp