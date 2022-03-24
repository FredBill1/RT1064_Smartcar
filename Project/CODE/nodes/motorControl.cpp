#include "nodes.hpp"
//
#include "devices.hpp"

static void motorControlEntry() {
    static SerialIO::TxUtil<float, 4, true> encoderXfer("encoder", 20);
    encoderL1.update(), encoderL2.update(), encoderR1.update(), encoderR2.update();
    if (encoderXfer.txFinished()) {
        encoderXfer.setAll(encoderL1.get(), encoderL2.get(), encoderR1.get(), encoderR2.get());
        wireless.send(encoderXfer);
    }
}

bool motorControlNode() { return FuncTimer(motorControlEntry, "motorControl", Param::MotorControlPeriod); }