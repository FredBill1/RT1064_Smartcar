#ifndef _apriltag_visualization_hpp
#define _apriltag_visualization_hpp

#include <cstdint>

#include "apriltag/apriltag.hpp"

namespace imgProc {
namespace apriltag {

void show_grayscale(const uint8_t* img);

void show_threshim(const QuadImg_t& img);

void plot(int_fast32_t i, int_fast32_t j, uint16_t color = 0xF800, int_fast32_t size = 2);

void show_unionfind();

void show_cluster(const List_pt_t& cluster, uint16_t color = 0xF800, int32_t delay = 0);

void show_clusters(const clusters_t& clusters);

void show_quads(quads_t& quads);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_visualization_hpp