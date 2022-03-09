#ifndef _apriltag_globalVariables_hpp
#define _apriltag_globalVariables_hpp

#include <cstdint>

#include "apriltag/apriltag.hpp"
#include "apriltag/config.hpp"
#include "apriltag/internal/Hashmap.hpp"

namespace imgProc {
namespace apriltag {

struct threshold_buf {
    uint8_t im_max[TN][TM], im_min[TN][TM];
    uint8_t im_max2[TN][TM], im_min2[TN][TM];
};
extern Unionfind_t uf;
extern threshold_buf thresholdbuf;
extern QuadImg_t threshim;
extern uint8_t hashmapbuf[sizeof(Hashmap)];

}  // namespace apriltag
}  // namespace imgProc
#endif  // _apriltag_globalVariables_hpp