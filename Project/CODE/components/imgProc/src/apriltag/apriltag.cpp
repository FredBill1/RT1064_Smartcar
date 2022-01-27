#include "apriltag/apriltag.hpp"

#include "apriltag/internal/Quick_decode.hpp"

namespace imgProc {
namespace apriltag {

void apriltag_family::init(int maxhamming, bool static_allocate) { quick_decode::init(*this, maxhamming, static_allocate); }

}  // namespace apriltag
}  // namespace imgProc