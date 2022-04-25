// https://github.com/psiorx/ADRC.git

#include "controller/LADRC.hpp"

#include <rtthread.h>

#include "LADRC_Impl.hpp"
using namespace Eigen;

namespace controller {

constexpr int Order = 1;

using Impl = LADRC_Impl<T, Order>;

LADRC::LADRC() {
    pimpl = rt_malloc(sizeof(Impl));
    pimpl = new (pimpl) Impl();
}
LADRC::LADRC(T wc, T wo, T b0, T dt) : LADRC() { setParameters(wc, wo, b0, dt); }
LADRC::~LADRC() {
    ((Impl*)pimpl)->~Impl();
    rt_free(pimpl);
}
void LADRC::setParameters(T wc, T wo, T b0, T dt) { ((Impl*)pimpl)->setParameters(wc, wo, b0, dt); }
void LADRC::reset() { ((Impl*)pimpl)->reset(); }
T LADRC::update(T u, T y, T y_desired) { return ((Impl*)pimpl)->update(u, y, y_desired); }

}  // namespace controller