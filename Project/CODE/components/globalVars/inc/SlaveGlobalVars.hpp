#ifndef _globalVars_SlaveGlobalVars_hpp
#define _globalVars_SlaveGlobalVars_hpp

#include <cstdint>

class SlaveGlobalVars {
 public:
    enum State : uint8_t {
        IDLE,
        RESET,
        A4,
        RECT,
    };

 private:
    State _state = IDLE;

 public:
    State get_state() const;
    void set_state(State state);
    static const char* state_str(State state);
};

extern SlaveGlobalVars slaveGlobalVars;

#endif  // _globalVars_SlaveGlobalVars_hpp