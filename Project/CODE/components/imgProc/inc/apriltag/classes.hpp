#ifndef _apriltag_classes_hpp
#define _apriltag_classes_hpp

#include "BinaryImg.hpp"
#include "apriltag/config.hpp"
#include "apriltag/internal/Unionfind.hpp"

namespace imgProc {
namespace apriltag {

using QuadImg_t = imgProc::QuadImg<N, M>;
using Unionfind_t = Unionfind<int32_t, N * M>;

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_classes_hpp