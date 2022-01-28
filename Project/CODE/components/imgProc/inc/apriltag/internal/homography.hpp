#ifndef _apriltag_homography_hpp
#define _apriltag_homography_hpp

namespace imgProc {
namespace apriltag {

void homography_compute2(double dst[3][3], double c[4][4]);

void homography_project(const double H[3][3], double x, double y, double *ox, double *oy);

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_homography_hpp