#ifndef _apriltag_visualization_hpp
#define _apriltag_visualization_hpp

#include <cstdint>

#include "apriltag/classes.hpp"

namespace imgProc {
namespace apriltag {

void plot(int_fast32_t i, int_fast32_t j, uint16_t color = 0xF800);

void show_unionfind();

void show_clusters(const clusters_t& clusters);

void show_quads(quads_t& quads);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_visualization_hpp