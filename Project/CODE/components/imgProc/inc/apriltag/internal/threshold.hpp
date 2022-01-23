#ifndef _threshold_hpp
#define _threshold_hpp

#include "apriltag/classes.hpp"
#include "apriltag/config.hpp"

namespace imgProc {
namespace apriltag {

void threshold(uint8_t* src, QuadImg& dst);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _threshold_hpp