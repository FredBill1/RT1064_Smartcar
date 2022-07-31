#include "ArmDrv.hpp"

#include <rtthread.h>

#include "devices.hpp"

void ArmDrv::initial_pose() const {}
void ArmDrv::pick() const {}
void ArmDrv::before_place() const {}
void ArmDrv::place(int index) const {}
void ArmDrv::drop(int index) const {}