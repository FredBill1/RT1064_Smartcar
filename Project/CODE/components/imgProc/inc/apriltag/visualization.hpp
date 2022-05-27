#ifndef _apriltag_visualization_hpp
#define _apriltag_visualization_hpp

#include <cstdint>

#include "apriltag/apriltag.hpp"

namespace imgProc {
namespace apriltag {

void show_grayscale(const uint8_t* img);

void show_threshim(const QuadImg_t& img);

void plot(int_fast32_t i, int_fast32_t j, uint16_t color = 0xF800, int_fast32_t size = 2);

void plotImg(uint8_t* img, int_fast32_t i, int_fast32_t j, uint16_t color = 0xF800, int_fast32_t size = 2);

void lineImg(uint8_t* img, int_fast32_t i0, int_fast32_t j0, int_fast32_t i1, int_fast32_t j1, uint16_t color,
             int_fast32_t size = 1);

void plotChar(uint8_t* img, int_fast32_t i, int_fast32_t j, char dat);

void plotInt(uint8_t* img, int_fast32_t i, int_fast32_t j, int32_t dat, int_fast32_t len = 2, bool atCenter = false);

void plot_tag_det(uint8_t* img, const apriltag_detection& det);

void show_plot_grayscale(const uint8_t* img);

void show_unionfind();

void show_cluster(const List_pt_t& cluster, uint16_t color = 0xF800, int32_t delay = 0);
void show_clusterImg(uint8_t* img, const List_pt_t& cluster, uint16_t color = 0xF800, int32_t delay = 0);

void show_clusters(const clusters_t& clusters);
void show_clustersImg(uint8_t* img, const clusters_t& clusters);

void show_quads(quads_t& quads);
void show_quadsImg(uint8_t* img, quads_t& quads);

void plot_rect(uint8_t* img, rect& r, bool show_magnitude = false);
void plot_rects(uint8_t* img, rects_t& rects, bool show_magnitude = false);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_visualization_hpp