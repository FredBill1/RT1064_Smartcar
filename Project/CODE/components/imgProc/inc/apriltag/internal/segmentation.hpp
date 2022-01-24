#ifndef _apriltag_segmentation_hpp
#define _apriltag_segmentation_hpp

#include "apriltag/classes.hpp"

namespace imgProc {
namespace apriltag {

void unionfind_connected(const QuadImg_t& img);

clusters_t* gradient_clusters(const QuadImg_t& img);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_segmentation