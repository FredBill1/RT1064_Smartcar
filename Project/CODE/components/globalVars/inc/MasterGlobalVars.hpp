#ifndef _globalVars_MasterGlobalVars_hpp
#define _globalVars_MasterGlobalVars_hpp

#include <rtthread.h>

class MasterGlobalVars {
 public:
    MasterGlobalVars();

 private:
    rt_event coord_recv_event;

 public:
    bool wait_for_coord_recv(rt_int32_t timeout = RT_WAITING_FOREVER);
    void send_coord_recv(int cnt, const float* coords);
    void get_coord_recv(int& cnt, float* coords);
};

extern MasterGlobalVars masterGlobalVars;

#endif  // _globalVars_MasterGlobalVars_hpp