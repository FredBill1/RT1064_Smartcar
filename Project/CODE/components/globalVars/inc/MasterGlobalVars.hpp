#ifndef _globalVars_MasterGlobalVars_hpp
#define _globalVars_MasterGlobalVars_hpp

#include <rtthread.h>

#include "edge_detect/A4Detect.hpp"

class MasterGlobalVars {
 public:
    MasterGlobalVars();

 private:
    bool reset_flag = false;

 public:
    void signal_reset();
    bool reset_requested();

 private:
    rt_event coord_recv_event;

 public:
    int coords_cnt;
    imgProc::apriltag::float_t coords[imgProc::edge_detect::target_coords_maxn + 1][2];  // start from index 1
    bool wait_for_coord_recv(rt_int32_t timeout = RT_WAITING_FOREVER);
    void send_coord_recv(int cnt, const float* coords);
    void get_coord_recv();

 private:
    bool _rectTargetEnabled = false;
    uint64_t _rectStartTimestamp_us, _rectCooldown_us;
    bool _rectStarted;
    float _rectTarget[3];  // [x, y, 与当前所推断距离的最大值]

 public:
    bool get_rectTarget(float target[3], uint64_t timestamp_us);
    void send_rectTarget(bool enabled, uint64_t timestamp_us = 0, int64_t cooldown_us = 0, const float target[3] = nullptr);

 private:
    rt_event art_snapshot_event;
    rt_event art_result_event;
    uint8_t _art_cur_index = 0;
    uint8_t _art_result;

 public:
    uint8_t art_results[imgProc::edge_detect::target_coords_maxn + 1];
    bool wait_art_snapshot(int index = 0, rt_int32_t timeout = RT_WAITING_FOREVER);
    void send_art_snapshot();
    bool wait_art_result(uint8_t& result, rt_int32_t timeout = RT_WAITING_FOREVER);
    void send_art_result(uint8_t result);
};

extern MasterGlobalVars masterGlobalVars;

#endif  // _globalVars_MasterGlobalVars_hpp