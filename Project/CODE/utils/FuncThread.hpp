#ifndef _Thread_hpp
#define _Thread_hpp

#include <rtthread.h>

#include <functional>

namespace Thread {

constexpr rt_uint8_t main_thread_priority = RT_MAIN_THREAD_PRIORITY;
constexpr rt_uint8_t terminal_thread_priority = FINSH_THREAD_PRIORITY;

constexpr rt_uint8_t highest_priority = 0;
constexpr rt_uint8_t lowest_priority = RT_THREAD_PRIORITY_MAX - 2;
constexpr rt_uint8_t default_priority = terminal_thread_priority + 1;

}  // namespace Thread

bool FuncThread(std::function<void()> func, const char* name, rt_uint32_t stack_size = 2048,
                rt_uint8_t priority = Thread::default_priority, rt_uint32_t tick = 20);

bool FuncTimer(std::function<void()> func, const char* name, rt_uint32_t period_tick);

#endif  // _Thread_hpp