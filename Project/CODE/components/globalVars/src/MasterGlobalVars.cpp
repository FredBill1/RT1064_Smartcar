#include "MasterGlobalVars.hpp"

#include <algorithm>

#include "Systick.hpp"
#include "edge_detect/A4Detect.hpp"
#include "utils/InterruptGuard.hpp"
using namespace imgProc::edge_detect;

MasterGlobalVars::MasterGlobalVars() {
    rt_event_init(&coord_recv_event, "coord_recv_event", RT_IPC_FLAG_PRIO);
    rt_event_init(&art_snapshot_event, "art_snapshot_event", RT_IPC_FLAG_PRIO);
    rt_event_init(&art_result_event, "art_result_event", RT_IPC_FLAG_PRIO);
}

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
    default: return "NULL";
    }
}

bool MasterGlobalVars::get_rectTarget(float target[3], uint64_t timestamp_us) {
    InterruptGuard guard;
    if (!_rectTargetEnabled) return false;
    if (!_rectStarted) {
        if (systick.get_diff_us(_rectStartTimestamp_us, timestamp_us) >= _rectCooldown_us) _rectStarted = true;
        else
            return false;
    }

    std::copy(_rectTarget, _rectTarget + 3, target);
    return true;
}
void MasterGlobalVars::send_rectTarget(bool enabled, uint64_t timestamp_us, int64_t cooldown_us, const float target[3]) {
    InterruptGuard guard;
    _rectTargetEnabled = enabled;
    if (!enabled) return;

    _rectStarted = false;
    _rectStartTimestamp_us = timestamp_us;
    _rectCooldown_us = cooldown_us;
    std::copy(target, target + 3, _rectTarget);
}

bool MasterGlobalVars::wait_art_snapshot(rt_int32_t timeout) {
    return rt_event_recv(&art_snapshot_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL) == RT_EOK;
}
void MasterGlobalVars::send_art_snapshot() { rt_event_send(&art_snapshot_event, 1); }
bool MasterGlobalVars::wait_art_result(uint8_t& result, rt_int32_t timeout) {
    if (rt_event_recv(&art_result_event, 1, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL) != RT_EOK) return false;
    {
        InterruptGuard guard;
        result = _art_result;
    }
    return true;
}
void MasterGlobalVars::send_art_result(uint8_t result) {
    {
        InterruptGuard guard;
        _art_result = result;
    }
    rt_event_send(&art_result_event, 1);
}

MasterGlobalVars masterGlobalVars;