#include "ArmDrv.hpp"

#include <rtthread.h>

#include "devices.hpp"

void ArmDrv::reset() const {
    for (int i = 0; i < 4; ++i) magnets[i].set(true);
    initial_pose();
}

void ArmDrv::initial_pose() const {
    magnets[3].set(true);
    srv_l.set(150);
    srv_r.set(90);
}
void ArmDrv::pick() const {
    srv_l.set(53);
    srv_r.set(40);
    rt_thread_mdelay(550);
}
void ArmDrv::before_place() const {
    srv_l.set(120);
    srv_r.set(189);
    rt_thread_mdelay(700);
}
void ArmDrv::place(int index) const {
    switch (index) {
    case 0: srv_l.set(104), srv_r.set(223); break;
    case 1: srv_l.set(122), srv_r.set(211); break;
    case 2: srv_l.set(142), srv_r.set(191); break;
    }
    rt_thread_mdelay(400);
    magnets[3].set(false);
    rt_thread_mdelay(300);
}
void ArmDrv::drop(int index) const {
    if (!(0 <= index && index < 3)) return;
    magnets[index].set(false);
    rt_thread_mdelay(100);
}