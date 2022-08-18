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
        srv_l.set(120);
        srv_r.set(172);
    }
    rt_thread_mdelay(700);
}
void ArmDrv::place(int index) const {
    switch (index) {
    case 0: {
        {
            InterruptGuard guard;
            srv_l.set(110);
            srv_r.set(208);
        }
        rt_thread_mdelay(200);
        {
            InterruptGuard guard;
            srv_l.set(106);
            srv_r.set(224);
        }
        rt_thread_mdelay(200);
    } break;
    case 1: {
        {
            InterruptGuard guard;
            srv_l.set(110);
            srv_r.set(205);
        }
        rt_thread_mdelay(200);
        {
            InterruptGuard guard;
            srv_l.set(135);
            srv_r.set(205);
        }
        rt_thread_mdelay(200);
    } break;
    case 2: {
        {
            InterruptGuard guard;
            srv_l.set(130);
            srv_r.set(175);
        }
        rt_thread_mdelay(150);
        {
            InterruptGuard guard;
            srv_l.set(132);
            srv_r.set(195);
        }
        rt_thread_mdelay(100);
        {
            InterruptGuard guard;
            srv_l.set(146);
            srv_r.set(200);
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