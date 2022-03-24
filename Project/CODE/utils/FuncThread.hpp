#ifndef _Thread_hpp
#define _Thread_hpp

#include <rtthread.h>

#include <functional>

bool FuncThread(std::function<void()> func, const char* name, rt_uint32_t stack_size = 2048,
                rt_uint8_t priority = (RT_THREAD_PRIORITY_MAX * 2) / 3, rt_uint32_t tick = 20);

bool FuncTimer(std::function<void()> func, const char* name, rt_uint32_t period_tick);

#endif  // _Thread_hpp