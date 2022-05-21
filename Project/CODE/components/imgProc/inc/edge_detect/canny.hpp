#ifndef _imgProc_edge_detect_canny_hpp
#define _imgProc_edge_detect_canny_hpp

#include <cstdint>

namespace imgProc {
namespace edge_detect {

typedef struct gvec {
    uint16_t t;
    uint16_t g;
} gvec_t;

gvec_t* canny(uint8_t* src, int low_thresh, int high_thresh);

}  // namespace edge_detect
}  // namespace imgProc

#endif  // _imgProc_edge_detect_canny_hpp