#ifndef _apriltag_fit_quad_hpp
#define _apriltag_fit_quad_hpp

#include "apriltag/apriltag.hpp"
#include "edge_detect/canny.hpp"
#include "imgProc/common.hpp"

namespace imgProc {
namespace apriltag {

bool fit_quad(List_pt_t& cluster, apriltag_family* tf, quad& quad, uint8_t* im);

quads_t* fit_quads(clusters_t& clusters, apriltag_family* tf, uint8_t* im, bool clear = false);

bool fit_quad_simple(Coordinate cluster[], int sz, quad& quad, const edge_detect::gvec_t* g);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_fit_quad_hpp