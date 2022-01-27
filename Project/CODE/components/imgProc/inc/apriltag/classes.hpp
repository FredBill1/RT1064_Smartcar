#ifndef _apriltag_classes_hpp
#define _apriltag_classes_hpp

#include <forward_list>
#include <unordered_map>
#include <utility>

#include "BinaryImg.hpp"
#include "apriltag/apriltag.hpp"
#include "apriltag/config.hpp"
#include "apriltag/internal/StaticBuffer.hpp"
#include "apriltag/internal/Unionfind.hpp"

namespace imgProc {
namespace apriltag {

using QuadImg_t = imgProc::QuadImg<N / quad_decimate, M / quad_decimate>;
using Unionfind_t = Unionfind<int32_t, N * M / (quad_decimate * quad_decimate)>;

using ID_t = uint32_t;

using List_pt_alloc_t = StaticAllocator<pt>;
using List_pt_t = std::forward_list<pt, List_pt_alloc_t>;

using clusters_alloc_t = StaticAllocator<List_pt_t*>;
using clusters_t = std::forward_list<List_pt_t*, clusters_alloc_t>;

using quads_alloc_t = StaticAllocator<quad>;
using quads_t = std::forward_list<quad, quads_alloc_t>;

using detections_alloc_t = StaticAllocator<apriltag_detection*>;
using detections_t = std::forward_list<apriltag_detection*, detections_alloc_t>;

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_classes_hpp