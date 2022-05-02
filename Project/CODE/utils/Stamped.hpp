#ifndef _utils_Stamped_hpp
#define _utils_Stamped_hpp

#include <cstdint>

template <class U> struct Stamped {
    uint64_t timestamp_us;
    U data;
};

#endif  // _utils_Stamped_hpp