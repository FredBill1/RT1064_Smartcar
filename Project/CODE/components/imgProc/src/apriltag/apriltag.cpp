#include "apriltag/apriltag.hpp"

#include "apriltag/internal/Quick_decode.hpp"

namespace imgProc {
namespace apriltag {

void apriltag_family::init(int maxhamming) { quick_decode::init(*this, maxhamming); }

}  // namespace apriltag
}  // namespace imgProc