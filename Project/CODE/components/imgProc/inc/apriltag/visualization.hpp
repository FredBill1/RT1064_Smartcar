#ifndef _apriltag_visualization_hpp
#define _apriltag_visualization_hpp

#include <cstdint>

#include "apriltag/apriltag.hpp"

namespace imgProc {
namespace apriltag {

void show_grayscale(const uint8_t* img);

void show_threshim(const QuadImg_t& img);

void plot(int_fast32_t i, int_fast32_t j, uint16_t color = 0xF800, int_fast32_t size = 2);

void plotImg(uint8_t* img, int_fast32_t i, int_fast32_t j, uint16_t color, int_fast32_t size = 2);

void lineImg(uint8_t* img, int_fast32_t i0, int_fast32_t j0, int_fast32_t i1, int_fast32_t j1, uint16_t color,
             int_fast32_t size = 1);

void plot_tag_det(uint8_t* img, apriltag_detection& det, uint16_t color);

void show_plot_grayscale(const uint8_t* img);

void show_unionfind();

void show_cluster(const List_pt_t& cluster, uint16_t color = 0xF800, int32_t delay = 0);

void show_clusters(const clusters_t& clusters);

void show_quads(quads_t& quads);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_visualization_hpp