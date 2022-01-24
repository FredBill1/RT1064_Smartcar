#ifndef _apriltag_UnionBuffer_hpp
#define _apriltag_UnionBuffer_hpp

#include <cstdint>

#include "apriltag/classes.hpp"
#include "apriltag/config.hpp"
#include "apriltag/internal/Hashmap.hpp"

namespace imgProc {
namespace apriltag {

union UnionBuffer {
    struct threshold {
        uint8_t im_max[TN][TM], im_min[TN][TM];
        uint8_t im_max2[TN][TM], im_min2[TN][TM];
    } threshold;

    struct segmentation {
        Unionfind_t uf;
        uint8_t hashmapbuf[sizeof(Hashmap)];
    } segmentation;
};

extern UnionBuffer unionBuffer;

}  // namespace apriltag
}  // namespace imgProc
#endif  // _apriltag_UnionBuffer_hpp