#ifndef _globalVars_MasterGlobalVars_hpp
#define _globalVars_MasterGlobalVars_hpp

#include <rtthread.h>

#include "edge_detect/A4Detect.hpp"

class MasterGlobalVars {
 public:
    MasterGlobalVars();

 private:
    rt_event coord_recv_event;

 public:
    int coords_cnt;
    imgProc::apriltag::float_t coords[imgProc::edge_detect::target_coords_maxn + 1][2];
    bool wait_for_coord_recv(rt_int32_t timeout = RT_WAITING_FOREVER);
    void send_coord_recv(int cnt, const float* coords);
    void get_coord_recv();

 public:
    enum State : uint8_t {
        IDLE,
        RESET,
        GET_COORDS,
        SOLVE_TSP,
        NAVIGATION,
    };

 private:
    State _state = IDLE;

 public:
    State get_state() const;
    void set_state(State state);
    static const char* state_str(State state);

 private:
    bool _rectTargetEnabled = false;
    float _rectTarget[2];

 public:
    bool get_rectTarget(float target[2]) const;
    void send_rectTarget(bool enabled, const float target[2] = nullptr);

 private:
    rt_event art_snapshot_event;
    rt_event art_result_event;
    uint8_t _art_result;

 public:
    bool wait_art_snapshot(rt_int32_t timeout = RT_WAITING_FOREVER);
    void send_art_snapshot();
    bool wait_art_result(uint8_t& result, rt_int32_t timeout = RT_WAITING_FOREVER);
    void send_art_result(uint8_t result);
};

extern MasterGlobalVars masterGlobalVars;

#endif  // _globalVars_MasterGlobalVars_hpp