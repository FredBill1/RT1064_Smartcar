#ifndef _imgProc_edge_detect_canny_hpp
#define _imgProc_edge_detect_canny_hpp

#include <cstdint>

namespace imgProc {
namespace edge_detect {

void canny(uint8_t* src, int low_thresh, int high_thresh);

}  // namespace edge_detect
}  // namespace imgProc

#endif  // _imgProc_edge_detect_canny_hpp