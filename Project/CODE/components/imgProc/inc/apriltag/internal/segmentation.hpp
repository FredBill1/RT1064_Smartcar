#ifndef _apriltag_segmentation_hpp
#define _apriltag_segmentation_hpp

#include "apriltag/apriltag.hpp"

namespace imgProc {
namespace apriltag {

void unionfind_connected(const QuadImg_t& img);

clusters_t* gradient_clusters(const QuadImg_t& img, const uint8_t* orig_im = nullptr);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_segmentation