#include "MasterGlobalVars.hpp"

#include "edge_detect/A4Detect.hpp"
#include "utils/InterruptGuard.hpp"

using namespace imgProc::edge_detect;

MasterGlobalVars::MasterGlobalVars() { rt_event_init(&coord_recv_event, "coord_recv_event", RT_IPC_FLAG_PRIO); }

bool MasterGlobalVars::wait_for_coord_recv(rt_int32_t timeout) {
    return rt_event_recv(&coord_recv_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL) == RT_EOK;
}

void MasterGlobalVars::send_coord_recv(int cnt, const float* coords) {
    {
        ScheduleGuard guard;
        target_coords_cnt = cnt;
        rt_memcpy(target_coords_corr, coords, sizeof(target_coords_corr[0]) * cnt);
    }
    rt_event_send(&coord_recv_event, 1);
}

void MasterGlobalVars::get_coord_recv() {
    ScheduleGuard guard;
    coords_cnt = target_coords_cnt + 1;
    rt_memcpy(coords + 1, target_coords_corr, sizeof(target_coords_corr[0]) * coords_cnt);
}

MasterGlobalVars::State MasterGlobalVars::get_state() const {
    InterruptGuard guard;
    return _state;
}

void MasterGlobalVars::set_state(State state) {
    InterruptGuard guard;
    _state = state;
}

const char* MasterGlobalVars::state_str(State state) {
    switch (state) {
    case IDLE: return "IDLE";
    case RESET: return "RST ";
    case GET_COORDS: return "GETC";
    case SOLVE_TSP: return "TSP ";
    case NAVIGATION: return "NAV ";
    }
    return "NULL";
}

MasterGlobalVars masterGlobalVars;