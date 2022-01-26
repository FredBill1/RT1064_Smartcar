#ifndef _apriltag_Graymodel_hpp
#define _apriltag_Graymodel_hpp

namespace imgProc {
namespace apriltag {

struct graymodel {
    double A[3][3];
    double B[3];
    double C[3];
    graymodel();
    void add(double x, double y, double gray);
    void solve();
    double interpolate(double x, double y);
};

}  // namespace apriltag
}  // namespace imgProc

#endif  // _apriltag_Graymodel_hpps