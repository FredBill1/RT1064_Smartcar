#ifndef _apriltag_Graymodel_hpp
#define _apriltag_Graymodel_hpp

#include "apriltag/config.hpp"

namespace imgProc {
namespace apriltag {

struct graymodel {
    float_t A[3][3];
    float_t B[3];
    float_t C[3];
    graymodel();
    void add(float_t x, float_t y, float_t gray);
    void solve();
    float_t interpolate(float_t x, float_t y);
};

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_Graymodel_hpps