#ifndef _nodes_masterConfig_hpp
#define _nodes_masterConfig_hpp

#include <rtthread.h>

constexpr int mainloop_timeout = 500;
constexpr rt_tick_t art_snapshot_timeout_ms = 1000;
constexpr rt_tick_t grab_srv_down_delay_ms = 150;
constexpr rt_tick_t magnet_drop_delay_ms = 30;
constexpr rt_tick_t magnet_drop_wait_ms = 350;
constexpr float art_cam_dist = 0.30;
constexpr bool use_art = true;

#define slave_uart uart2
#define art_uart uart5
#define upload_uart uart4

#endif