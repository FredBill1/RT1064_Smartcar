#ifndef _devices_hpp
#define _devices_hpp

#include "BaseDriver.hpp"
#include "Encoder.hpp"
#include "GPIO.hpp"
#include "ICM20948.hpp"
#include "IPS.hpp"
#include "MotorCtrl.hpp"
#include "MotorDRV.hpp"
#include "MoveBase.hpp"
#include "QTimer.hpp"
#include "SerialIO.hpp"
#include "Systick.hpp"
#include "controller/LADRC.hpp"
#include "pose_kalman/LocalPlanner.hpp"
#include "pose_kalman/PoseKalman.hpp"

//
#include "PinConfig.h"

extern const GPIO led;
extern const GPIO beep;

extern const GPIO btn_c4;
extern const GPIO btn_c26;
extern const GPIO btn_c27;
extern const GPIO btn_c31;
extern const GPIO switch_d27;
extern const GPIO switch_d4;

extern const GPIO master_key[5];
extern const GPIO master_switch[3];
extern const GPIO slave_key[5];
extern const GPIO slave_switch[3];

extern Systick systick;

extern SerialIO uart2;
extern SerialIO uart3;
extern SerialIO uart4;
extern SerialIO uart5;
extern SerialIO wireless;

extern ICM20948 imu;

extern IPS ips;

extern MotorDRV motorDrvL1, motorDrvL2, motorDrvR1, motorDrvR2;

extern const QTimer qtimerL1, qtimerL2, qtimerR1, qtimerR2;
extern Encoder encoderL1, encoderL2, encoderR1, encoderR2;

extern controller::LADRC controllerL1, controllerL2, controllerR1, controllerR2;
extern MotorCtrl motorCtrlL1, motorCtrlL2, motorCtrlR1, motorCtrlR2;
extern BaseDriver baseDriver;

namespace pose_kalman {
extern PoseKalman kf;
extern LocalPlanner localPlanner;
}  // namespace pose_kalman

extern MoveBase moveBase;

void initDevices();

#endif  // _devices_hpp