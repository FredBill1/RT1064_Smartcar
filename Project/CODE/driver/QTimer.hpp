#ifndef _QTimer_hpp
#define _QTimer_hpp

extern "C" {
#include "zf_qtimer.h"
}

class QTimer {
 private:
    const QTIMERN_enum Qtimern;
    const QTIMER_PIN_enum A, B;

 public:
    QTimer(QTIMERN_enum qtimern, QTIMER_PIN_enum phaseA, QTIMER_PIN_enum phaseB) : Qtimern(qtimern), A(phaseA), B(phaseB) {}
    void init() const { qtimer_quad_init(Qtimern, A, B); }
    int16 get() const { return qtimer_quad_get(Qtimern, A); }
    void reset() const { qtimer_quad_clear(Qtimern, A); }
};

#endif  // _QTimer_hpp