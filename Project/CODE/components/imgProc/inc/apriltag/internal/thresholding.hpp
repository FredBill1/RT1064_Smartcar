#ifndef _thresholding_hpp
#define _thresholding_hpp

#include "apriltag/TagQuadImg.hpp"
#include "apriltag/config.hpp"

namespace imgProc {
namespace apriltag {

void threshold(uint8_t* src, QuadImg& dst);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _thresholding_hpp