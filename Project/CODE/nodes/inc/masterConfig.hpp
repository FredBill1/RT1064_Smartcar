#ifndef _nodes_masterConfig_hpp
#define _nodes_masterConfig_hpp

#include <rtthread.h>

constexpr int mainloop_timeout = 500;
constexpr rt_tick_t art_snapshot_timeout_ms = 1000;
constexpr float initial_position[3]{0, 0, 0};
constexpr float art_cam_dist = 0.2;
constexpr bool use_art = false;

#define slave_uart uart3
#define art_uart uart4

#endif