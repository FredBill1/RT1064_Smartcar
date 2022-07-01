#ifndef _globalVars_MasterGlobalVars_hpp
#define _globalVars_MasterGlobalVars_hpp

#include <rtthread.h>

#include <bitset>

#include "RectConfig.hpp"
#include "edge_detect/A4Detect.hpp"

class MasterGlobalVars {
 public:
    MasterGlobalVars();
    void clear_events();

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
    std::bitset<imgProc::edge_detect::target_coords_maxn + 1> coord_valid;
    bool wait_for_coord_recv(rt_int32_t timeout = RT_WAITING_FOREVER);
    void send_coord_recv(int cnt, const float* coords);
    void get_coord_recv();

 private:
    bool _rectEnabled = false;
    float _rectCoords[imgProc::apriltag::max_rect_cnt][2];
    float _rectRecvingState[3];
    float _rectMaxDistErrorSquared;
    uint8_t _rectCnt = 0;
    uint64_t _rectTimestamp_us;

 public:
    void send_rects_enabled(bool enabled, float maxDistErrorSquared = 0);
    void send_rects(const float state[3], const float* rects, int cnt, uint64_t timestamp_us);
    void get_rects(float state[3], float* rects, int& cnt, float& maxDistErrorSquared, uint64_t& timestamp_us);

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