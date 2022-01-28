#ifndef _apriltag_quad_decode_hpp
#define _apriltag_quad_decode_hpp

#include "apriltag/apriltag.hpp"

namespace imgProc {
namespace apriltag {
detections_t *decode_quads(const apriltag_family &family, uint8_t *im, quads_t &quads);
}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_quad_decode_hpp