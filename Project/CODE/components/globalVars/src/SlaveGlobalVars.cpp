#include "SlaveGlobalVars.hpp"

#include "utils/InterruptGuard.hpp"

SlaveGlobalVars::State SlaveGlobalVars::get_state() const {
    InterruptGuard guard;
    return _state;
}

void SlaveGlobalVars::set_state(State state) {
    InterruptGuard guard;
    _state = state;
}

const char* SlaveGlobalVars::state_str(State state) {
    switch (state) {
    case IDLE: return "IDLE";
    case RESET: return "RST ";
    case A4: return "A4  ";
    case RECT: return "RECT";
    default: return "NULL";
    }
}

SlaveGlobalVars slaveGlobalVars;