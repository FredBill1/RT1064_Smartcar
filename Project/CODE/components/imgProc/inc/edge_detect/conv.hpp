#ifndef _imgProc_edge_detect_conv_hpp
#define _imgProc_edge_detect_conv_hpp

#include <cstdint>

namespace imgProc {
namespace edge_detect {

// Image kernels
extern const int8_t kernel_gauss_3[9];
extern const int8_t kernel_gauss_5[25];
extern const int kernel_laplacian_3[9];
extern const int kernel_high_pass_3[9];

void sepconv3(uint8_t* src, const int8_t* krn, const float m, const int b);

}  // namespace edge_detect
}  // namespace imgProc

#endif  // _imgProc_edge_detect_conv_hpp