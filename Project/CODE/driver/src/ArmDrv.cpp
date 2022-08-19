#include "ArmDrv.hpp"

#include <rtthread.h>

#include "devices.hpp"
#include "utils/InterruptGuard.hpp"

void ArmDrv::reset() const {
    for (int i = 0; i < 4; ++i) magnets[i].set(true);
    initial_pose();
}

void ArmDrv::initial_pose() const {
    magnets[3].set(true);
    {
        InterruptGuard guard;
        srv_l.set(150);
        srv_r.set(90);
    }
}
void ArmDrv::pick() const {
    {
        InterruptGuard guard;
        srv_l.set(53);
        srv_r.set(40);
    }
    rt_thread_mdelay(550);
}
void ArmDrv::before_place() const {
    {
        InterruptGuard guard;
        srv_l.set(105);
        srv_r.set(195);
    }
    rt_thread_mdelay(700);
}
void ArmDrv::place(int index) const {
    switch (index) {
    case 0: {
        {
            InterruptGuard guard;
            srv_l.set(97);
            srv_r.set(210);
        }
        rt_thread_mdelay(200);
        srv_r.set(241);
        rt_thread_mdelay(200);
    } break;
    case 1: {
        srv_r.set(219);
        rt_thread_mdelay(135);
        srv_l.set(123);
        rt_thread_mdelay(135);
        srv_l.set(115);
        rt_thread_mdelay(135);
    } break;
    case 2: {
        srv_l.set(127);
        rt_thread_mdelay(150);
        {
            InterruptGuard guard;
            srv_l.set(132);
            srv_r.set(211);
        }
        rt_thread_mdelay(100);
        {
            InterruptGuard guard;
            srv_l.set(138);
            srv_r.set(203);
        }
        rt_thread_mdelay(200);
    } break;
    }
    magnets[3].set(false);
    rt_thread_mdelay(300);
}
void ArmDrv::drop(int index) const {
    if (!(0 <= index && index < 3)) return;
    magnets[index].set(false);
    rt_thread_mdelay(100);
}