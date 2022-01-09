#ifndef _Encoder_hpp
#define _Encoder_hpp

extern "C" {
#include "zf_qtimer.h"
}

class QTimer {
 private:
    const QTIMERN_enum Qtimern;
    const QTIMER_PIN_enum A, B;

 public:
    QTimer(QTIMERN_enum qtimern, QTIMER_PIN_enum phaseA, QTIMER_PIN_enum phaseB)
        : Qtimern(qtimern), A(phaseA), B(phaseB) {}
    void init() { qtimer_quad_init(Qtimern, A, B); }
    int16 get() { return qtimer_quad_get(Qtimern, A); }
    void reset() { qtimer_quad_clear(Qtimern, A); }
};

#endif  // _Encoder_hpp