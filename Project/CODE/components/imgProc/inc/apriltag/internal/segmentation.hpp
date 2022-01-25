#ifndef _apriltag_segmentation_hpp
#define _apriltag_segmentation_hpp

#include "apriltag/classes.hpp"

namespace imgProc {
namespace apriltag {

void unionfind_connected(const QuadImg_t& img);

clusters_t* gradient_clusters(const QuadImg_t& img);

void show_unionfind();

void show_clusters(const clusters_t& clusters);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_segmentation