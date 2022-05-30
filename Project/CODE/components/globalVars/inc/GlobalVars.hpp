#ifndef _globalVars_globalVars_hpp
#define _globalVars_globalVars_hpp

#include <rtthread.h>

class GlobalVars {
 public:
    GlobalVars();

 private:
    rt_event coord_recv_event;

 public:
    bool wait_for_coord_recv(rt_int32_t timeout = RT_WAITING_FOREVER);
    void send_coord_recv(int cnt, const float* coords);
    void get_coord_recv(int& cnt, float* coords);
};

extern GlobalVars globalVars;

#endif  // _globalVars_globalVars_hpp