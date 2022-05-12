#ifndef _apriltag_quad_decode_hpp
#define _apriltag_quad_decode_hpp

#include "apriltag/apriltag.hpp"

namespace imgProc {
namespace apriltag {
void refine_edges(uint8_t *im_orig, quad *quad);
detections_t *decode_quads(const apriltag_family &family, uint8_t *im, quads_t &quads, bool debug = false);
}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_quad_decode_hpp