#ifndef _std_msgs_hpp
#define _std_msgs_hpp

#include <cstdint>

namespace rosRT {
namespace msgs {

using float32 = float;
using float64 = double;

using float_t = float32;

struct Header {
    uint32_t stamp;
};

}  // namespace msgs
}  // namespace rosRT

#endif  // _std_msgs_hpp