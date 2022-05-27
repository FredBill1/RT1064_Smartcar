#ifndef _imgProc_apriltag_line_magnitude_hpp
#define _imgProc_apriltag_line_magnitude_hpp

#include "apriltag/apriltag.hpp"
#include "apriltag/config.hpp"

namespace imgProc {
namespace apriltag {

float_t pixel_magnitude(const uint8_t* img, int x, int y);
rects_t& rects_magnitude(uint8_t* img, quads_t& quads, float min_magnitude);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _imgProc_apriltag_line_magnitude_hpp