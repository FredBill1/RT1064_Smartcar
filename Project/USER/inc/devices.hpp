#ifndef _devices_hpp
#define _devices_hpp

#include "Encoder.hpp"
#include "ICM20948.hpp"
#include "IPS.hpp"
#include "MotorDRV.hpp"
#include "SerialIO.hpp"

//
#include "PinConfig.h"

extern SerialIO uart2;
extern SerialIO uart3;
extern SerialIO uart4;
extern SerialIO uart5;
extern SerialIO wireless;

extern ICM20948 imu;

extern IPS ips;

extern MotorDRV motorDrvL1, motorDrvL2, motorDrvR1, motorDrvR2;

#endif  // _devices_hpp