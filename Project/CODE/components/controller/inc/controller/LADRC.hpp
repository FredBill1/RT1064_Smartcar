// https://github.com/psiorx/ADRC.git

#ifndef _controller_ADRC_hpp
#define _controller_ADRC_hpp

#include "controller/config.hpp"

namespace controller {
class LADRC {
    void* pimpl;

 public:
    LADRC();
    LADRC(T wc, T wo, T b0, T dt);
    ~LADRC();
    void setParameters(T wc, T wo, T b0, T dt);
    void reset();
    T update(T u, T y, T y_desired);
};

}  // namespace controller

#endif  // _controller_ADRC_hpp