#ifndef _StaticBuffer_hpp
#define _StaticBuffer_hpp

#include "apriltag/config.hpp"

namespace imgProc {
namespace apriltag {

union UnionBuffer {
    struct threshold {
        uint8_t im_max[TN][TM], im_min[TN][TM];
        uint8_t im_max2[TN][TM], im_min2[TN][TM];
    } threshold;
};

extern UnionBuffer unionBuffer;

}  // namespace apriltag
}  // namespace imgProc

#endif  //_StaticBuffer_hpp