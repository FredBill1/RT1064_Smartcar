#include "ArmDrv.hpp"

#include <rtthread.h>

#include "devices.hpp"

void ArmDrv::initial_pose() const {
    srv_l.set(127);
    srv_r.set(132);
}
void ArmDrv::pick() const {
    magnets[3].set(true);
    srv_l.set(62);
    srv_r.set(40);
    rt_thread_mdelay(420);
    srv_l.set(48);
    rt_thread_mdelay(50);
}
void ArmDrv::before_place() const {
    srv_l.set(120);
    srv_r.set(196);
    rt_thread_mdelay(500);
}
void ArmDrv::place(int index) const {
    switch (index) {
    case 0: srv_l.set(96), srv_r.set(236); break;
    case 1: srv_l.set(113), srv_r.set(221); break;
    case 2: srv_l.set(130), srv_r.set(211); break;
    }
    rt_thread_mdelay(300);
    magnets[3].set(false);
    rt_thread_mdelay(200);
}
void ArmDrv::drop(int index) const {}